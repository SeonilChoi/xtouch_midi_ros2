# xtouch_midi_ros2

ROS 2 bridge for the Behringer X-Touch Extender and `ros2_motor_manager`.

The `xtouch_midi` node reads X-Touch fader MIDI values, loads the motor layout
from `ros2_motor_manager/config/example_canopen_zeroerr.yaml` by default, and
publishes `motor_status_msgs/msg/MotorStatus` commands on `motor_command`.

It also keeps the raw X-Touch topics available:

| Topic | Type | Payload |
| --- | --- | --- |
| `/xtouch/fader/ch0` ... `/xtouch/fader/ch7` | `std_msgs/msg/Int32` | Raw 14-bit fader value, `0..16383` |
| `/xtouch/touch/ch0` ... `/xtouch/touch/ch7` | `std_msgs/msg/Bool` | `true` while the fader is touched |
| `/motor_command` | `motor_status_msgs/msg/MotorStatus` | Motor command generated from the fader value |

## Install Dependencies

This package uses the MIDI dependency files from `ros2_midi/requirements`.
Run this once from the workspace source tree:

```bash
cd <colcon_ws>/src/ros2
bash ros2_midi/requirements/install_requirements.sh
```

The script installs the apt packages listed in
`ros2_midi/requirements/apt_packages.txt`, including `librtmidi-dev` and
`libasound2-dev`. ROS 2 Humble must already be installed.

Check that the X-Touch is visible to ALSA:

```bash
amidi -l
```

The output should include an X-Touch or Behringer MIDI port.

## Build

From the workspace root:

```bash
cd <colcon_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_status_msgs ros2_motor_manager xtouch_midi
source install/setup.bash
```

If the old `ros2_midi/src/xtouch_midi` package is also in the same workspace,
build only the new package paths to avoid the duplicate `xtouch_midi` package
name:

```bash
colcon build --symlink-install \
  --base-paths src/ros2/motion_system_ros2 src/ros2/xtouch_midi_ros2/xtouch_midi \
  --packages-select motor_status_msgs ros2_motor_manager xtouch_midi
```

## Run

Start the motor manager first, or run it in another terminal:

```bash
source /opt/ros/humble/setup.bash
source <colcon_ws>/install/setup.bash
ros2 launch ros2_motor_manager motor_manager_node.launch.py
```

Then start the X-Touch MIDI bridge:

```bash
source /opt/ros/humble/setup.bash
source <colcon_ws>/install/setup.bash
ros2 launch xtouch_midi xtouch_node.launch.py
```

The launch file uses this config by default:

```text
ros2_motor_manager/config/example_canopen_zeroerr.yaml
```

To use another motor config:

```bash
ros2 launch xtouch_midi xtouch_node.launch.py config_file:=/absolute/path/to/config.yaml
```

## Verify

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source <colcon_ws>/install/setup.bash

ros2 topic echo /motor_command
ros2 topic echo /xtouch/fader/ch0
ros2 topic echo /xtouch/touch/ch0
```

Move the first fader. The node maps fader channel `chN` to controller index
`N` from the YAML config.

## Troubleshooting

- `No MIDI input port matching ...`: check USB power/cable and run `amidi -l`.
- `Permission denied`: add the user to the `audio` group, then log out and back in:
  `sudo usermod -aG audio $USER`
- No `/motor_command` output: the node publishes only when a fader sends MIDI input.
