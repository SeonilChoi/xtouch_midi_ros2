#include "xtouch_midi/xtouch_node.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <stdexcept>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <std_msgs/msg/int8_multi_array.hpp>

namespace
{

constexpr std::size_t kNumChannels = 8;
constexpr uint8_t kFaderTouchNoteStart = 104;
constexpr uint8_t kFaderTouchNoteEnd = kFaderTouchNoteStart + kNumChannels - 1;
constexpr int32_t kFaderValueMax = 16383;
constexpr auto kDebounceInterval = std::chrono::milliseconds(100);
constexpr auto kDebounceTick = std::chrono::milliseconds(50);

constexpr uint16_t kCwNewSetPointZeroerr = 0x103F;
constexpr uint16_t kCwNewSetPointMinas = 0x003F;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRpmToRadPerSec = 2.0 * kPi / 60.0;

std::string to_upper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

std::string to_lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool looks_like_xtouch(const std::string & port_name)
{
  const std::string up = to_upper(port_name);
  return up.find("X-TOUCH") != std::string::npos ||
         up.find("XTOUCH") != std::string::npos ||
         up.find("BEHRINGER") != std::string::npos;
}

template<typename T>
T required_as(const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key]) {
    throw std::runtime_error("Missing '" + std::string(key) + "' in " + context);
  }
  return node[key].as<T>();
}

}  // namespace

namespace xtouch_midi
{

template<typename Port>
void XTouchNode::open_matching_port(Port & port, const char * direction)
{
  const unsigned int n = port.getPortCount();
  RCLCPP_INFO(get_logger(), "Scanning %u MIDI %s port(s)...", n, direction);

  for (unsigned int i = 0; i < n; ++i) {
    const std::string name = port.getPortName(i);
    RCLCPP_INFO(get_logger(), "  [%s %u] %s", direction, i, name.c_str());
    if (looks_like_xtouch(name)) {
      port.openPort(i);
      RCLCPP_INFO(
        get_logger(), "Connected MIDI %s port: '%s'", direction, name.c_str());
      return;
    }
  }

  throw std::runtime_error(
    std::string("No MIDI ") + direction +
    " port matching 'X-Touch'/'XTOUCH'/'Behringer' found. "
    "Is the device connected and powered on?");
}

XTouchNode::XTouchNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("xtouch_node", options)
{
  const std::string config_file =
    declare_parameter<std::string>("config_file", "");
  const std::string command_topic =
    declare_parameter<std::string>("command_topic", "motor_command");
  publish_raw_topics_ = declare_parameter<bool>("publish_raw_topics", true);

  if (config_file.empty()) {
    throw std::runtime_error(
      "Parameter 'config_file' is empty. Pass a ros2_motor_manager YAML file.");
  }

  motor_infos_ = load_motor_infos(config_file);
  if (motor_infos_.empty()) {
    throw std::runtime_error("No motor controllers were found in " + config_file);
  }

  if (publish_raw_topics_) {
    for (std::size_t i = 0; i < kNumChannels; ++i) {
      const std::string suffix = "ch" + std::to_string(i);
      fader_pubs_[i] = create_publisher<std_msgs::msg::Int32>(
        "/xtouch/fader/" + suffix, 10);
      touch_pubs_[i] = create_publisher<std_msgs::msg::Bool>(
        "/xtouch/touch/" + suffix, 10);
    }
  }

  motor_command_pub_ = create_publisher<MotorStatus>(
    command_topic, rclcpp::QoS(1).best_effort());

  midi_in_ = std::make_unique<RtMidiIn>();
  midi_in_->ignoreTypes(true, true, true);
  open_matching_port(*midi_in_, "input");

  midi_out_ = std::make_unique<RtMidiOut>();
  open_matching_port(*midi_out_, "output");

  midi_in_->setCallback(&XTouchNode::midi_trampoline, this);

  debounce_tick_ = create_wall_timer(
    kDebounceTick, std::bind(&XTouchNode::tick_debounce, this));

  RCLCPP_INFO(
    get_logger(),
    "xtouch_node ready. Loaded %zu controller(s) from '%s'; publishing "
    "MotorStatus commands on '%s'.",
    motor_infos_.size(), config_file.c_str(), command_topic.c_str());
}

XTouchNode::~XTouchNode()
{
  if (midi_in_) {
    midi_in_->cancelCallback();
    if (midi_in_->isPortOpen()) {
      midi_in_->closePort();
    }
  }
  if (midi_out_ && midi_out_->isPortOpen()) {
    midi_out_->closePort();
  }
}

void XTouchNode::midi_trampoline(
  double /* timestamp */,
  std::vector<unsigned char> * msg,
  void * user_data)
{
  if (msg == nullptr || user_data == nullptr) {
    return;
  }
  static_cast<XTouchNode *>(user_data)->on_midi(*msg);
}

void XTouchNode::on_midi(const std::vector<unsigned char> & bytes)
{
  if (bytes.size() < 3) {
    return;
  }

  const uint8_t status = bytes[0] & 0xF0;
  const uint8_t channel = bytes[0] & 0x0F;
  const uint8_t d1 = bytes[1];
  const uint8_t d2 = bytes[2];

  if (status == 0xE0 && channel < kNumChannels) {
    const int32_t value = static_cast<int32_t>((d2 << 7) | d1);

    if (publish_raw_topics_ && fader_pubs_[channel]) {
      std_msgs::msg::Int32 m;
      m.data = value;
      fader_pubs_[channel]->publish(m);
    }

    publish_motor_command(channel, value);

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      last_fader_value_[channel] = value;
      debounce_deadline_[channel] =
        std::chrono::steady_clock::now() + kDebounceInterval;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500, "fader[%u] = %d", channel, value);
    return;
  }

  const bool is_note_on = (status == 0x90);
  const bool is_note_off = (status == 0x80);
  if ((is_note_on || is_note_off) &&
    d1 >= kFaderTouchNoteStart && d1 <= kFaderTouchNoteEnd)
  {
    const std::size_t ch = d1 - kFaderTouchNoteStart;
    const bool touched = is_note_on && d2 > 0;

    if (publish_raw_topics_ && touch_pubs_[ch]) {
      std_msgs::msg::Bool m;
      m.data = touched;
      touch_pubs_[ch]->publish(m);
    }

    RCLCPP_INFO(get_logger(), "touch[%zu] = %s", ch, touched ? "down" : "up");
    return;
  }
}

void XTouchNode::tick_debounce()
{
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t ch = 0; ch < kNumChannels; ++ch) {
    int32_t value = 0;
    bool fire = false;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (debounce_deadline_[ch].has_value() &&
        now >= *debounce_deadline_[ch])
      {
        value = last_fader_value_[ch];
        debounce_deadline_[ch].reset();
        fire = true;
      }
    }
    if (fire) {
      send_fader_pitch_bend(static_cast<uint8_t>(ch), value);
    }
  }
}

void XTouchNode::send_fader_pitch_bend(uint8_t ch, int32_t value)
{
  value = std::clamp<int32_t>(value, 0, kFaderValueMax);
  std::vector<unsigned char> bytes = {
    static_cast<unsigned char>(0xE0 | (ch & 0x0F)),
    static_cast<unsigned char>(value & 0x7F),
    static_cast<unsigned char>((value >> 7) & 0x7F),
  };

  try {
    midi_out_->sendMessage(&bytes);
  } catch (const RtMidiError & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MIDI out failed on ch %u: %s", ch, e.what());
  }
}

std::vector<XTouchNode::MotorInfo> XTouchNode::load_motor_infos(
  const std::string & config_file) const
{
  YAML::Node root = YAML::LoadFile(config_file);
  if (!root) {
    throw std::runtime_error("Failed to load motor config: " + config_file);
  }

  const YAML::Node drivers_node = root["drivers"];
  if (!drivers_node || !drivers_node.IsSequence()) {
    throw std::runtime_error("Invalid or missing 'drivers' in " + config_file);
  }

  std::unordered_map<int, DriverInfo> drivers;
  for (const auto & driver_node : drivers_node) {
    const int id = required_as<int>(driver_node, "id", "drivers[]");
    DriverInfo info;
    info.id = static_cast<uint8_t>(id);
    info.lower = required_as<double>(driver_node, "lower", "drivers[]");
    info.upper = required_as<double>(driver_node, "upper", "drivers[]");
    info.speed = required_as<double>(driver_node, "speed", "drivers[]");
    info.rated_torque =
      required_as<double>(driver_node, "rated_torque", "drivers[]");
    info.type = to_lower(required_as<std::string>(driver_node, "type", "drivers[]"));
    drivers.emplace(id, info);
  }

  const YAML::Node masters_node = root["masters"];
  if (!masters_node || !masters_node.IsSequence()) {
    throw std::runtime_error("Invalid or missing 'masters' in " + config_file);
  }

  std::map<int, MotorInfo> motors_by_index;
  for (const auto & master_node : masters_node) {
    const YAML::Node slaves_node = master_node["slaves"];
    if (!slaves_node || !slaves_node.IsSequence()) {
      throw std::runtime_error("Invalid or missing 'slaves' in masters[]");
    }

    for (const auto & slave_node : slaves_node) {
      const int controller_index =
        required_as<int>(slave_node, "controller_index", "slaves[]");
      const int driver_id = required_as<int>(slave_node, "driver_id", "slaves[]");
      const int profile_mode =
        required_as<int>(slave_node, "profile_mode", "slaves[]");

      if (controller_index < 0 || controller_index > 255) {
        throw std::runtime_error("controller_index is out of uint8 range.");
      }

      const auto driver_iter = drivers.find(driver_id);
      if (driver_iter == drivers.end()) {
        throw std::runtime_error(
          "No driver entry for driver_id " + std::to_string(driver_id));
      }

      MotorInfo info;
      info.controller_index = static_cast<uint8_t>(controller_index);
      info.profile_mode = static_cast<int8_t>(profile_mode);
      info.driver = driver_iter->second;

      const auto [_, inserted] = motors_by_index.emplace(controller_index, info);
      if (!inserted) {
        throw std::runtime_error(
          "Duplicate controller_index " + std::to_string(controller_index));
      }
    }
  }

  std::vector<MotorInfo> result;
  result.reserve(motors_by_index.size());
  int expected_index = 0;
  for (const auto & [controller_index, info] : motors_by_index) {
    if (controller_index != expected_index) {
      throw std::runtime_error(
        "controller_index values must be dense from 0 for motor_manager.");
    }
    result.push_back(info);
    ++expected_index;
  }

  return result;
}

XTouchNode::MotorStatus XTouchNode::make_empty_motor_status() const
{
  MotorStatus msg;
  const std::size_t n = motor_infos_.size();

  msg.number_of_target_interfaces.assign(n, 0);
  msg.target_interface_id.resize(n);
  msg.controller_index.resize(n);
  msg.controlword.assign(n, 0);
  msg.statusword.assign(n, 0);
  msg.errorcode.assign(n, 0);
  msg.position.assign(n, 0.0);
  msg.velocity.assign(n, 0.0);
  msg.torque.assign(n, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    msg.controller_index[i] = motor_infos_[i].controller_index;
  }

  return msg;
}

void XTouchNode::publish_motor_command(std::size_t channel, int32_t fader_value)
{
  if (channel >= motor_infos_.size()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring fader channel %zu; no controller in config.", channel);
    return;
  }

  const MotorInfo & motor = motor_infos_[channel];
  const std::size_t idx = motor.controller_index;
  MotorStatus msg = make_empty_motor_status();

  if (idx >= msg.controller_index.size()) {
    RCLCPP_WARN(
      get_logger(), "Controller index %zu is outside MotorStatus array.", idx);
    return;
  }

  switch (motor.profile_mode) {
    case 0: {
      msg.number_of_target_interfaces[idx] = 2;
      msg.target_interface_id[idx].data = std::vector<int8_t>{0, 1};
      msg.controlword[idx] = controlword_for_driver(motor.driver.type);
      msg.position[idx] = scale_fader(
        fader_value, motor.driver.lower, motor.driver.upper);
      break;
    }
    case 1: {
      msg.number_of_target_interfaces[idx] = 1;
      msg.target_interface_id[idx].data = std::vector<int8_t>{2};
      const double rpm = scale_fader(
        fader_value, -motor.driver.speed, motor.driver.speed);
      msg.velocity[idx] = rpm * kRpmToRadPerSec;
      break;
    }
    case 2: {
      msg.number_of_target_interfaces[idx] = 1;
      msg.target_interface_id[idx].data = std::vector<int8_t>{3};
      msg.torque[idx] = scale_fader(
        fader_value, -motor.driver.rated_torque, motor.driver.rated_torque);
      break;
    }
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported profile_mode %d for controller %u.",
        motor.profile_mode, motor.controller_index);
      return;
  }

  motor_command_pub_->publish(msg);
}

double XTouchNode::scale_fader(
  int32_t fader_value, double lower, double upper) const
{
  const double normalized =
    static_cast<double>(std::clamp<int32_t>(fader_value, 0, kFaderValueMax)) /
    static_cast<double>(kFaderValueMax);
  return lower + (upper - lower) * normalized;
}

uint16_t XTouchNode::controlword_for_driver(const std::string & driver_type) const
{
  const std::string type = to_lower(driver_type);
  if (type == "zeroerr") {
    return kCwNewSetPointZeroerr;
  }
  if (type == "minas") {
    return kCwNewSetPointMinas;
  }

  RCLCPP_WARN(
    get_logger(), "Unknown driver type '%s'; using zeroerr set-point controlword.",
    driver_type.c_str());
  return kCwNewSetPointZeroerr;
}

}  // namespace xtouch_midi

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<xtouch_midi::XTouchNode>();
    rclcpp::spin(node);
  } catch (const RtMidiError & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("xtouch_node"), "RtMidi error: %s", e.what());
    rclcpp::shutdown();
    return 2;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("xtouch_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
