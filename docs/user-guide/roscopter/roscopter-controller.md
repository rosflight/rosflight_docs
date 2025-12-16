# Controller

## Responsibility
The responsibility of the `controller` is to

1. produce low-level commands from high-level commands
2. Maintain a state machine to manage autonomous takeoff

## Interface with ROScopter
| ![Diagram of ROScopter architecture](../images/roscopter_architecture.svg) |
|:--:|
|*Reference diagram of the ROScopter architecture*|

The `controller` interfaces with the rest of ROScopter using publishers and subscribers.

| Subscriber name | Message type | Description |
| :--- | :--- | :--- |
| `estimated_state` | `roscopter_msgs/State` | Estimated state for the vehicle (needs everything) |
| `status` | `rosflight_msgs/Status` | Status message from the ROSflight firmware (needs to know when RC control is active or when the vehicle is armed) |
| `high_level_command` | `roscopter_msgs/ControllerCommand` | Input controller command |

| Publisher name | Message type | Description |
| :--- | :--- | :--- |
| `command` | `rosflight_msgs/Command` | Output commands to the downstream ROSflight firmware controller |

To summarize, the ROScopter `controller` takes in the estimated state and high-level command setpoints and computes low-level command setpoints that are sent to the ROSflight firmware on the flight control unit.
Note that the output command messages are sent to ROSflight firmware (on the [flight control unit](../overview.md)) via the `rosflight_io` node.

## Using the `controller`/implementation details
Many different control schemes exist for multirotor vehicles.
Since application code (i.e. code that you write) has slightly different outputs, the ROScopter controller has the architecture shown in the following figure.

| ![ROScopter cascaded architecture](../images/roscopter_and_firmware_controllers.svg) |
| :---: |
| Diagram of the ROScopter controller architecture and how the cascaded controller chain interacts with the ROSflight firmware controller |

The above figure shows the cascaded architecture of the ROScopter controller.
In this diagram, the arrows represent how one controller output feeds into the next controller's input, all the way down to one of three basic control types.
For example, if the user passes control setpoints corresponding to Controller 0 (inertial-frame north, east, down position and yaw commands), Controller 0 converts them to Controller 3 inputs, which then converts to Controller 2 inputs, and so on.

The following table describes the inputs to each type of controller.

| Number | Name | Description of reference commands |
| :--- | :--- | :--- |
| 0  | NED-Pos Yaw | Inertial north, east, down (NED) positions and yaw |
| 1  | NE-Vel D-Pos YawR | Inertial N-E velocities, D position, and yaw rate |
| 2  | FRD-Accel YawR | Vehicle-1 frame[^1] (labeled front-right-down) accelerations and yaw rate |
| 3  | NED-Vel YawR | Inertial NED velocities and yaw rate |
| 4  | NE-Pos D-Vel Yaw | Inertial N-E velocities and yaw |
| 5  | Roll Pitch Yaw Throttle | Roll, pitch, yaw, and throttle |
| 6  | Roll Pitch YawR Throttle | Roll, pitch, yaw rate, and throttle |
| 7  | RollR PitchR YawR Throttle | Roll rate, pitch rate, yaw rate, throttle |
| 8  | Pass-through | Pass-through to ROSflight firmware mixer |
| 9 | Roll Pitch Yaw Thrust | Roll, pitch, yaw, and thrust to pass-through |
| 10 | Roll Pitch YawR Thrust | Roll, pitch, yaw rate, and thrust to pass-through |
| 11 | RollR PitchR YawR Thrust | Roll rate, pitch rate, yaw rate, and thrust to pass-through |

[^1]: R. W. Beard and T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, 2012, Princeton University Press, see also https://github.com/byu-magicc/mavsim_public.

!!! danger "Reference frames"
    Knowing which reference frame you are sending commands in is essential for safe operation.
    The `NED` frame in the table refers to the **inertial** north, east, and down reference frame.

    Thus, if I send commands to the controller 1 entrypoint, I am sending inertial north and east velocities and inertial down position, **regardless of how the aircraft is oriented**.

    Accelerations (controller 2), however, are in the **vehicle 1** frame[^1], which we call the FRD (front-right-down) frame in the above table.
    Because it is the vehicle 1 frame, front means whichever direction the vehicle is facing (but not that it does not matter how the vehicle is pitched or rolled).

    All of the roll, pitch, and yaw commands are defined in their usual reference frames.

The `controller`'s cascaded architecture allows users to "insert" control commands at many different levels, depending on the needs of their application code.
Each insertion point thus produces a different *controller chain*.

!!! note
    We'll call the sequence of controllers used as a *controller chain*.

    For example, inserting at Controller 4 would have the controller chain of Controller 4 -> Controller 3 -> Controller 2 -> Controller 6.

Each insertion point is chosen via the `mode` field in the `roscopter_msgs/ControllerCommand` message.
Thus, each ROS 2 message determines which controller it inserts at.
This means that different controller chains can be chosen at runtime, and can be mixed and matched throughout the flight.

!!! example "Using different controller chains during a flight"
    Since the insertion points are chosen via the `mode` field in each ROS 2 message sent to the `controller`, controller chains can be mixed and matched throughout the flight.

    For example, in the default implementation of ROScopter, the `controller` inserts commands at Controller 4 during takeoff and Controller 10 during the other portions of the flight.

As seen in the above ROScopter `controller` diagram, the output of the `controller` node is sent directly to the ROSflight firmware via the `rosflight_io` node (note that the `rosflight_io` node is not shown in the diagram).
The output of all controller chains needs to be one of the modes that the firmware controller can accept.
Thus, the output of the ROScopter controller is one of the following output types:
$$
    u_\text{angle} = [\phi^d, \theta^d, r^d, \delta_t^d]^T
$$
$$
    u_\text{rate} = [p^d, q^d, r^d, \delta_t^d]^T
$$
$$
    u_\text{pass-through} = [Q_x^d, Q_y^d, Q_z^d, T_z^d]^T
$$
where $\phi$, $\theta$, and $\psi$ are roll, pitch, and yaw commands, $p$, $q$, $r$ are roll rate, pitch rate, and yaw rate, and $\delta_t \in [0,1]$ is the throttle setpoint.
$Q$ and $T$ are the body-frame torques and forces, respectively.

### Controller State Machine
The `controller` node also has a simple state machine to manage safe takeoff and landing.

| ![ROScopter controller state machine](../images/roscopter_state_machine.svg) |
| :---: |
| Diagram of the `controller`'s state machine.

The `controller` state machine starts in the disarmed state until the user arms the vehicle.
Additionally, the state machine requires a valid command setpoint sent to the `high_level_command` topic.
This is determined by the `cmd_valid` field in the `roscopter_msgs/ControllerCommand` message definition.

The state machine then transitions to takeoff mode, where the vehicle will climb at a constant velocity until it reaches a particular height.
Once the vehicle reaches the desired height (within a given threshold), takeoff is considered complete and the vehicle transitions to position hold mode.

After a brief pause in position hold mode, the vehicle then enters "offboard control" mode, which means that the controller will output control commands to track the commands on the `high_level_command` topic.

During the takeoff and position hold states, the commands sent to the `high_level_command` topic are overridden.
This ensures that the vehicle is at a safe height before attempting to follow any path commands.
Note that if the `cmd_valid` field is set to `false` in messages from the `high_level_command` topic, then the offboard control state will transition to position hold.

The behavior of the state machine can be configured via [the `controller`'s parameters](#parameters-and-configuration).
This includes the takeoff down velocity, the height the controller will climb to, the threshold that the state machine considers the takeoff waypoint complete, and the time that the state machine will spend in takeoff before transitioning to offboard control mode.

## Parameters and configuration
The parameters associated with the `controller` are listed below.
Parameters that have notes/special considerations are discussed below the table.

| Parameter name | Parameter type | Description |
| :--- | :--- | :--- |
| `equilibrium_throttle` | `double` | Throttle value required to maintain hover. Between 0-1 |
| `gravity` | `double` | Gravity in $m/s^2$ |
| `mass` | `double` | Vehicle mass (in kg) |
| `max_descend_accel` | `double` | Maximum descent acceleration. **Applies to the acceleration controller** |
| `max_descend_rate` | `double` | Maximum down velocity. **Applies to the velocity PID controllers** |
| `max_pitch_deg` | `double` | Maximum pitch angle (in degrees). **Applies to the angle PID controllers.** |
| `max_pitch_rate_deg` | `double` | Maximum pitch rate (in degrees/sec). **Applies to the rate PID controllers.** |
| `max_pitch_torque` | `double` | Maximum pitch torque (in N-m). **Applies to the torque controllers.** |
| `max_roll_deg` | `double` | Maximum roll angle (in degrees). **Applies to the angle PID controllers.** |
| `max_roll_rate_deg` | `double` | Maximum roll rate (in degrees/sec). **Applies to the rate PID controllers.** |
| `max_roll_torque` | `double` | Maximum roll torque (in N-m). **Applies to the torque controllers.** |
| `max_throttle` | `double` | Maximum allowed throttle setpoint. Should be between 0-1 |
| `max_yaw_rate_deg` | `double` | Maximum yaw rate (in degrees/sec). **Applies to the rate PID controllers.** |
| `max_yaw_torque` | `double` | Maximum yaw torque (in N-m). **Applies to the torque PID controllers.** |
| `min_altitude_for_attitude_ctrl` | `double` | Minimum altitude before attitude control. Roll and pitch commands are zero when the vehicle is lower than this number. Helps to reduce crashes on takeoff. |
| `min_throttle` | `double` | Minimum throttle setpoint. Should be between 0-1 |
| `takeoff_d_pos` | `double` | Down position to achieve (in takeoff) before turning control over to the "offboard control". |
| `takeoff_d_vel` | `double` | Down velocity during takeoff (until vehicle reaches the `takeoff_d_pos` position. |
| `takeoff_height_threshold` | `double` | Radius (in m) that the vehicle must achieve around the `takeoff_d_pos` setpoint before the state machine considers takeoff to be complete. |
| `takeoff_landing_pos_hold_time` | `double` | Time (in seconds) the state machine will hold before switching to offboard control (after takeoff) or landing. Holding for a brief time before switching over helps with stability. |
| `tau` | `double` | Bandwidth of the dirty derivative (for the PID controllers). See the [ControlBook](https://github.com/byu-controlbook/controlbook_public). |

!!! danger "Max/min parameters"
    The parameters associated with max and min values (e.g. `max_descend_accel`) **only apply if the control loop associated with those parameters is run**.

    For example, the `max_descend_rate` parameter is a saturation limit on the down velocity controller, meaning that the output of the down velocity controller is saturated to never be greater than this number.
    However, if the velocity controller is not run (meaning a controller lower in [the controller chain](#using-the-controllerimplementation-details) is used), then the velocity controller will not run and this max velocity limit will not be respected.

    Please note which control loops you are using before tuning these parameters, as some will not be used based on where you are inserting into the control chain.

The `max_throttle` parameter is usually set to be lower than 1 to reserve some actuator effort for attitude control.
The `min_throttle` parameter is usually set to a value higher than 0 to avoid the motors "shutting off" for particular maneuvers.

!!! warning
    Make sure the `mass` parameter is correct and that it is the same between all nodes (e.g. the [`trajectory_follower`)](./roscopter-trajectory-follower.md).

The following parameters are listed for completeness.
Each of the PID control types from the [implementation details](#using-the-controllerimplementation-details) section has 3 gains associated with it.

Note that the control loops that are used depends on where the high-level command setpoints are inserted into the controller chain.
Thus, not all of the PID gains below will need to be tuned when using ROScopter.
Make sure to check which gains are used based on where you are inserting command setpoints.

| | Parameter name | Parameter name | Parameter name |
| :--- | :--- | :--- | :--- |
| **Rate to torque** | `pitch_rate_to_torque_kd` | `roll_rate_to_torque_kd` | `yaw_rate_to_torque_kd` |
| | `pitch_rate_to_torque_ki` | `roll_rate_to_torque_ki` | `yaw_rate_to_torque_ki` |
| | `pitch_rate_to_torque_kp` | `roll_rate_to_torque_kp` | `yaw_rate_to_torque_kp` |
| **Angle to torque** | `pitch_to_torque_kd` |  `roll_to_torque_kd` | `yaw_to_torque_kd` |
| | `pitch_to_torque_ki` |  `roll_to_torque_ki` | `yaw_to_torque_ki` |
| | `pitch_to_torque_kp` |  `roll_to_torque_kp` | `yaw_to_torque_kp` |
| **Position to velocity** | `pos_n_to_vel_kd` | `pos_d_to_vel_kd` | `pos_e_to_vel_kd` |
| | `pos_n_to_vel_ki` | `pos_d_to_vel_ki` | `pos_e_to_vel_ki` |
| | `pos_n_to_vel_kp` | `pos_d_to_vel_kp` | `pos_e_to_vel_kp` |
| **Velocity to acceleration** | `vel_d_to_accel_kd` | `vel_e_to_accel_kd` | `vel_n_to_accel_kd` |
| | `vel_d_to_accel_ki` | `vel_e_to_accel_ki` | `vel_n_to_accel_ki` |
| | `vel_d_to_accel_kp` | `vel_e_to_accel_kp` | `vel_n_to_accel_kp` |
| **Yaw to yaw rate** | `yaw_to_rate_kd` | | |
| | `yaw_to_rate_ki` | | |
| | `yaw_to_rate_kp` | | |

!!! note
    Each PID gain is named according to what it takes in and what it returns.

    For example, the derivative gain for the PID loop that takes in position commands and returns velocity commands in the down direction is called `pos_d_to_vel_kd`.

