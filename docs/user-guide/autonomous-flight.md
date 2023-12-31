# Autonomous Flight

To perform autonomous flight with ROSflight, we need to send control commands from our companion computer to the firmware. This can be done with ROSplane or ROScopter, which already have completed autonomy stacks developed specifically for ROSflight. We recommend starting with one of these autonomy stacks and building on them to suit your needs. If using a multirotor, follow the [ROScopter setup guide](roscopter-setup.md) to get started. If using a fixed-wing, follow the [ROSplane setup guide](rosplane-setup.md).

However, ROSplane and ROScopter are optional and an entirely different autonomy stack can be used if desired. To use a different autonomy stack, follow this guide.

## Provide Control from a Companion Computer

Control setpoints are sent to the flight controller by publishing to the `/command` topic that is advertised by the `rosflight_io` node. This topic accepts messages of type `rosflight_msgs/Command`, which have the following structure:

```
std_msgs/Header header
uint8 mode
uint8 ignore
float32 x
float32 y
float32 z
float32 f
```

The `header` field is a standard ROS2 message header. The `x`, `y`, `z`, and `f` fields are the control setpoint values, which are interpreted according to the `mode` and `ignore` fields.

The following table describes the different values the `mode` field can take, as well as how the setpoint values are interpreted for each of these modes:

| Value | Enum | x | y | z | F |
|-------|------|---|---|---|---|
| 0 | `MODE_PASS_THROUGH` | aileron deflection (-1 to 1) | elevator deflection (-1 to 1) | rudder deflection (-1 to 1) | throttle (0 to 1) |
| 1 | `MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE` | roll rate (rad/s) | pitch rate (rad/s) | yaw rate (rad/s) | throttle (0 to 1) |
| 2 | `MODE_ROLL_PITCH_YAWRATE_THROTTLE` | roll angle (rad) | pitch angle (rad) | yaw rate (rad/s) | throttle (0 to 1) |

The `MODE_PASS_THROUGH` mode is used for fixed-wing vehicles to directly specify the control surface deflections and throttle, while the `MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE` and `MODE_ROLL_PITCH_YAWRATE_THROTTLE` modes are used for multirotor vehicles to specify the attitude rates or angles, respectively.

The `ignore` field is used if you want to specify control setpoints for some, but not all, of the axes. For example, I may want to specify throttle setpoints to perform altitude hold, while still letting the RC pilot specify the attitude setpoints. The `ignore` field is a bitmask that can be populated by combining the following values:

| Value | Enum | Result |
|-------|------|--------|
| 0 | `IGNORE_NONE` | Ignore none of the fields (default) |
| 1 | `IGNORE_X` | Ignore the `x` field |
| 2 | `IGNORE_Y` | Ignore the `y` field |
| 4 | `IGNORE_Z` | Ignore the `z` field |
| 8 | `IGNORE_F` | Ignore the `F` field |

For the previous example, I would set the `ignore` field to a value of
```
ignore = IGNORE_X | IGNORE_Y | IGNORE_Z
```

The best practice is to use enum names rather than the actual numeric values for the `mode` and `ignore` fields. For example, to specify a multirotor attitude angle command in C++ I might have:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>

rosflight_msgs::msg::Command msg;
msg.header.stamp = node->get_clock()->now();
msg.mode = rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
msg.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;
msg.x = 0.0;
msg.y = 0.0;
msg.z = 0.0;
msg.f = 0.6;
```

In Python I might have:
```python
import rclpy
from rosflight_msgs.msg import Command

msg = Command()
msg.header.stamp = node.get_clock().now().to_msg()
msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
msg.ignore = Command.IGNORE_NONE
msg.x = 0.0
msg.y = 0.0
msg.z = 0.0
msg.f = 0.6
```
I would then publish this message to the `/command` topic to forward it to the embedded flight controller.

!!! note
    If the flight controller does not receive a new command for a defined period of time, it will ignore the old commands and revert to RC control. The length of this timeout period is set by the `OFFBOARD_TIMEOUT` parameter.
