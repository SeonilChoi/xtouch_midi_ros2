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
The paths below assume this workspace layout:

```text
<ws>/src/ros2/xtouch_midi_ros2
<ws>/src/ros2/ros2_midi
<ws>/src/ros2/motion_system_ros2
```

For this machine, `<ws>` is `/home/mini0/colcon_ws`.

Run this once from the ROS 2 source folder:

```bash
cd <ws>/src/ros2
bash ros2_midi/requirements/install_requirements.sh
```

The script installs the apt packages listed in
`ros2_midi/requirements/apt_packages.txt`, including `librtmidi-dev` and
`libasound2-dev`. ROS 2 Humble must already be installed.

Add your user to the `audio` group so the node can access ALSA MIDI devices
without `sudo`:

```bash
sudo usermod -aG audio $USER
```

Log out and back in after changing the group membership. To verify it:

```bash
groups
```

The output should include `audio`.

Check that the X-Touch is visible to ALSA:

```bash
amidi -l
```

The output should include an X-Touch or Behringer MIDI port.

## Build

From the workspace root:

```bash
cd <ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src/common src/lib src/ros2/motion_system_ros2 src/ros2/xtouch_midi_ros2/xtouch_midi \
  --packages-up-to ros2_motor_manager xtouch_midi
source install/setup.bash
```

The `--base-paths` argument intentionally excludes `src/ros2/ros2_midi/src/xtouch_midi`,
because that older reference package has the same package name: `xtouch_midi`.

## Run

Start the motor manager first, or run it in another terminal:

```bash
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash
ros2 launch ros2_motor_manager motor_manager_node.launch.py
```

Then start the X-Touch MIDI bridge:

```bash
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash
ros2 launch xtouch_midi xtouch_node.launch.py
```

The launch file uses this config by default:

```text
<ws>/install/ros2_motor_manager/share/ros2_motor_manager/config/example_canopen_zeroerr.yaml
```

To use another motor config:

```bash
ros2 launch xtouch_midi xtouch_node.launch.py config_file:=/absolute/path/to/config.yaml
```

Motor command mapping starts from the physical fader channel set by
`kChannelOffset` in `xtouch_node.cpp`. The current default maps physical `ch6`
to controller index `0`, `ch7` to controller index `1`, and so on.

## Verify

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash

ros2 topic echo /motor_command
ros2 topic echo /xtouch/fader/ch0
ros2 topic echo /xtouch/touch/ch0
```

Move the first fader. The node maps fader channel `chN` to controller index
`N - kChannelOffset` from the YAML config. Raw `/xtouch/fader/chN` and
`/xtouch/touch/chN` topics keep using the physical X-Touch channel number.

## Troubleshooting

- `No MIDI input port matching ...`: check USB power/cable and run `amidi -l`.
- `Permission denied`: confirm your user is in the `audio` group with `groups`,
  then log out and back in if it was just added.
- No `/motor_command` output: the node publishes only when a fader sends MIDI input.
