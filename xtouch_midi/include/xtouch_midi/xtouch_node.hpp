#ifndef XTOUCH_MIDI_XTOUCH_NODE_HPP_
#define XTOUCH_MIDI_XTOUCH_NODE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rtmidi/RtMidi.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <motor_status_msgs/msg/motor_status.hpp>

namespace xtouch_midi
{

class XTouchNode : public rclcpp::Node
{
public:
  using MotorStatus = motor_status_msgs::msg::MotorStatus;

  explicit XTouchNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~XTouchNode() override;

private:
  struct DriverInfo
  {
    uint8_t id{};
    double lower{};
    double upper{};
    double speed{};
    double rated_torque{};
    std::string type;
  };

  struct MotorInfo
  {
    uint8_t controller_index{};
    int8_t profile_mode{};
    DriverInfo driver;
  };

  template<typename Port>
  void open_matching_port(Port & port, const char * direction);

  static void midi_trampoline(
    double timestamp,
    std::vector<unsigned char> * msg,
    void * user_data);

  void on_midi(const std::vector<unsigned char> & bytes);
  void publish_pending_motor_command();
  void tick_debounce();
  void send_fader_pitch_bend(uint8_t ch, int32_t value);

  std::vector<MotorInfo> load_motor_infos(const std::string & config_file) const;
  MotorStatus make_empty_motor_status() const;
  bool fill_motor_command_target(
    MotorStatus & msg, std::size_t channel, int32_t fader_value);
  double scale_fader(int32_t fader_value, double lower, double upper) const;
  uint16_t controlword_for_driver(const std::string & driver_type) const;

  std::unique_ptr<RtMidiIn> midi_in_;
  std::unique_ptr<RtMidiOut> midi_out_;

  std::array<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr, 8> fader_pubs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, 8> touch_pubs_;
  rclcpp::Publisher<MotorStatus>::SharedPtr motor_command_pub_;

  std::vector<MotorInfo> motor_infos_;
  bool publish_raw_topics_{true};

  std::mutex state_mutex_;
  std::array<int32_t, 8> last_fader_value_{};
  std::array<bool, 8> fader_value_valid_{};
  bool motor_command_dirty_{false};
  std::array<std::optional<std::chrono::steady_clock::time_point>, 8>
    debounce_deadline_{};

  rclcpp::TimerBase::SharedPtr command_tick_;
  rclcpp::TimerBase::SharedPtr debounce_tick_;
};

}  // namespace xtouch_midi

#endif  // XTOUCH_MIDI_XTOUCH_NODE_HPP_
