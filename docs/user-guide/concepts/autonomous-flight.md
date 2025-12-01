# Autonomous Flight

To perform autonomous flight with ROSflight, we need to send control commands from our companion computer to the firmware. This can be done with ROSplane or ROScopter, which already have completed autonomy stacks developed specifically for ROSflight. We recommend starting with one of these autonomy stacks and building on them to suit your needs. If using a multirotor, follow the [ROScopter setup guide](roscopter-overview.md) to get started. If using a fixed-wing, follow the [ROSplane setup guide](rosplane-setup.md).

However, ROSplane and ROScopter are optional and an entirely different autonomy stack can be used if desired. To use a different autonomy stack, follow this guide.

## Provide Control from a Companion Computer

Control setpoints are sent to the flight controller by publishing to the `/command` topic that is advertised by the `rosflight_io` node. This topic accepts messages of type `rosflight_msgs/Command`, which have the following structure:

```
➜  ~ ros2 interface show rosflight_msgs/msg/Command
# Offboard control command message

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2

# ignore field bitmasks
uint8 IGNORE_NONE = 0
uint8 IGNORE_QX = 1
uint8 IGNORE_QY = 2
uint8 IGNORE_QZ = 4
uint8 IGNORE_FX = 8
uint8 IGNORE_FY = 16
uint8 IGNORE_FZ = 32

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
uint8 mode # offboard control mode for interpreting value fields
uint8 ignore # bitmask for ignore specific setpoint values
float32 qx
float32 qy
float32 qz
float32 fx
float32 fy
float32 fz
```

The `header` field is a standard ROS2 message header. The `q*`, and `f*` fields are the control setpoint values, which are interpreted according to the `mode` and `ignore` fields.

The following table describes the different values the `mode` field can take, as well as how the setpoint values are interpreted for each of these modes:

| Value | Enum | Description |
|-------|------|---|
| 0 | `MODE_PASS_THROUGH` | Passthrough directly to mixer. Units interpreted according to mixer |
| 1 | `MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE` | Passed to firmware rate controller. `q*` is in units of rad/s |
| 2 | `MODE_ROLL_PITCH_YAWRATE_THROTTLE` | Passed to firmware angle controller. `q*` is in units of rad |

The `MODE_PASS_THROUGH` mode is most often used for fixed-wing vehicles to directly specify the control surface deflections and throttle, while the `MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE` and `MODE_ROLL_PITCH_YAWRATE_THROTTLE` modes are used for multirotor vehicles to specify the attitude rates or angles, respectively.

The `ignore` field is used if you want to specify control setpoints for some, but not all, of the axes.
The `ignore` field is a bitmask that can be populated by combining the values in the `IGNORE_*` enum in the message definition.

!!! example

    For example, I may want to specify throttle setpoints to perform altitude hold, while still letting the RC pilot specify the attitude setpoints.
    To do this, I would set the `ignore` field to a value of
    ```
    ignore = IGNORE_QX | IGNORE_QY | IGNORE_FZ
    ```

The best practice is to use enum names rather than the actual numeric values for the `mode` and `ignore` fields. For example, to specify a multirotor attitude angle command in C++ I might have:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>

rosflight_msgs::msg::Command msg;
msg.header.stamp = node->get_clock()->now();
msg.mode = rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
msg.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;
msg.qx = 0.0;
msg.qy = 0.0;
msg.qz = 0.0;
msg.fz = 0.6;
```

In Python I might have:
```python
import rclpy
from rosflight_msgs.msg import Command

msg = Command()
msg.header.stamp = node.get_clock().now().to_msg()
msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
msg.ignore = Command.IGNORE_NONE
msg.qx = 0.0
msg.qy = 0.0
msg.qz = 0.0
msg.fz = 0.6
```
I would then publish this message to the `/command` topic to forward it to the embedded flight controller.

!!! note
    If the flight controller does not receive a new command for a defined period of time, it will ignore the old commands and revert to RC control. The length of this timeout period is set by the `OFFBOARD_TIMEOUT` parameter.
