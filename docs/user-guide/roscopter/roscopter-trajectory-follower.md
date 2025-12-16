# Trajectory Follower
## Responsibility
The responsibility of the `trajectory_follower` is to produce controller setpoints to follow a given trajectory.

Note that the `trajectory_follower` does not take in an entire trajectory, but rather a trajectory setpoint *at a particular instant in time*.

## Interface with ROScopter
| ![Diagram of ROScopter architecture](../images/roscopter_architecture.svg) |
|:--:|
|*Reference diagram of the ROScopter architecture*|

The `trajectory_follower` interfaces with the rest of ROScopter using publishers and subscribers.

| Subscriber name | Message type | Description |
| :--- | :--- | :--- |
| `estimated_state` | `roscopter_msgs/State` | Estimated state for the vehicle (needs position and orientation) |
| `status` | `rosflight_msgs/Status` | Status message from the ROSflight firmware (needs to know when RC control is active) |
| `trajectory_command` | `roscopter_msgs/TrajectoryCommand` | Input trajectory command |

| Publisher name | Message type | Description |
| :--- | :--- | :--- |
| `high_level_command` | `roscopter_msgs/ControllerCommand` | Output commands to the downstream ROScopter controller |

To summarize, the `trajectory_follower` takes in the estimated state (from the `estimator`) and trajectory setpoints (from the `path_manager`), and computes and publishes the commands to the `controller` node.

## Using the `trajectory_follower`/implementation details
The input trajectory commands are vectors of the form
$$
    u_\text{traj} = [p, \dot{p}, \ddot{p}, \psi, \dot{\psi}, \ddot{\psi}]^T,
$$
where $p$ and $\psi$ are the desired 3-D position and heading setpoints, respectively.
The derivative terms are feed-forward terms for each setpoint.

The output of the `trajectory_follower` is controller setpoints of the form
$$
    u_\text{out} = [\phi^c, \theta^c, r^c, T^c]
$$
where $\phi^c$ and $\theta^c$ are the desired roll and pitch in radians, respectively, and $r^c$ is the desired yaw rate in radians per second.
$T^c$ is the desired thrust, in Newtons.

The trajectory follower is based off of the ["Differential flatness based control of a rotorcraft for aggressive maneuvers"](https://ieeexplore.ieee.org/document/6095098) by Ferrin, et al[^1].
We refer the reader to that paper for information on the implementation and how it works.

[^1]: J. Ferrin, R. Leishman, R. Beard and T. McLain, "Differential flatness based control of a rotorcraft for aggressive maneuvers," 2011 IEEE/RSJ International Conference on Intelligent Robots and Systems, San Francisco, CA, USA, 2011, pp. 2688-2693, doi: 10.1109/IROS.2011.6095098.

There are a number of differences in the ROScopter `trajectory_follower` not present in the work by Ferrin, et al.

1. The ROScopter `trajectory_follower` uses four PID controllers (north, east, down, and yaw) as the nominal controllers, not an LQR controller (as seen in Fig. 2 of the paper).
This means that there are 4 sets of separate PID gains for each controller.

    !!! info
        The nominal controllers (the PID controllers) have integrators.
        To avoid integrator wind-up, the `trajectory_follower` clears the integrator values when the RC safety pilot has RC override.

        The `trajectory_follower` determines if the safety pilot has control of the vehicle using the `status` message.

1. Eq. (16)[^1] requires computing $\dot{\theta}$ (note this is not $q$!). The `trajectory_follower` assumes constant acceleration over the timestep to compute this derivative.
1. Before computing $\boldsymbol{z}$ in Eq. (13)[^1], we first saturate $u_{p_3}$ to avoid instabilities near the origin for Eq. (15)[^1].

!!! note
    Saturating $u_{p_3}$ is necessary to prevent poor performance when the trajectory follower tries to track large down setpoints.

    The total desired down acceleration is computed as
    $$
    u_{p_3} = \ddot{p}^c - g
    $$
    where $g=9.81$ is gravity.

    When $\ddot{p}^c$ approaches $+g$, then $u_{p_3}$ approaches $0$, and Eq. (15)[^1] returns large values of $\theta$ for small changes in $z_1$.
    Physically, this results in large, rapidly changing pitch commands, resulting in undesireable performance.
    Note that this can happen for large nominal (PID) controller output, but also for large feedforward input.

    To fix this, we need to make sure $u_{p_3}$ does not approach $0$, or equivalently, $\ddot{p}^c$ does not approach $+g$.
    Since free-fall is also usually not desireable (i.e. $\ddot{p}^c=0$), we saturate $u_{p_3}$ so that
    $$
    u_{p_3} \in [u_\text{max}, -\infty]
    $$
    where $u_\text{max}$ is a negative number controlled by the `max_commanded_down_accel_in_gs` parameter.

    Note that the `max_commanded_down_accel_in_gs` parameter **should be a negative number**, e.g. $-0.4$.
    This means that the maximum down command is $-0.4g$.
    Since we are using the NED frame, this translates to a minimum of $0.4g$ commanded output acceleration up, so a maximum $0.6g$ actual acceleration down.

### Trajectory Follower Services
The `trajectory_follower` offers the following service server:

| Service name | Interface type | Description |
| :--- | :--- | :--- |
| `clear_integrators` | `std_srvs/Trigger` | Clears the integrator values in the nominal PID controllers |


## Parameters and configuration
The parameters associated with the `trajectory_follower` are

| Parameter name | Parameter type | Description |
| :--- | :--- | :--- |
| `down_command_window` | `double` | If input down command to down controller is larger than this parameter, input command gets saturated to this value. |
| `gravity` | `double` | Gravity in $m/s^2$ |
| `mass` | `double` | Mass of the system in kg |
| `max_commanded_down_accel_in_gs` | `double` | Maximum down acceleration the `trajectory_follower` can command (in $g$'s). See the [implementation details](#using-the-trajectory_followerimplementation-details). |
| `tau` | `double` | Bandwidth of the dirty derivative (for the PID controllers). See the [ControlBook](https://github.com/byu-controlbook/controlbook_public). |
| `u_n_kp` | `double` | $k_p$ gain for the north PID controller |
| `u_n_ki` | `double` | $k_i$ gain for the north PID controller |
| `u_n_kd` | `double` | $k_d$ gain for the down PID controller |
| `u_e_kp` | `double` | $k_p$ gain for the east PID controller |
| `u_e_ki` | `double` | $k_i$ gain for the east PID controller |
| `u_e_kd` | `double` | $k_d$ gain for the east PID controller |
| `u_d_kp` | `double` | $k_p$ gain for the down PID controller |
| `u_d_ki` | `double` | $k_i$ gain for the down PID controller |
| `u_d_kd` | `double` | $k_d$ gain for the down PID controller |
| `yaw_to_rate_kp` | `double` | $k_p$ gain for the yaw angle to yaw rate PID controller |
| `yaw_to_rate_ki` | `double` | $k_i$ gain for the yaw angle to yaw rate PID controller |
| `yaw_to_rate_kd` | `double` | $k_d$ gain for the yaw angle to yaw rate PID controller |

!!! danger "Mass and gravity parameters"
    The `gravity` and `mass` parameters **must** be correct.
    Otherwise, the `trajectory_follower` will output incorrect controller commands.

    Make sure the `mass` parameter is the same between all other nodes.

As described in the [implementation details section](#using-the-trajectory_followerimplementation-details), there are 4 PID controllers, one for north, east, down position, and one for yaw to yaw-rate.
Each controller has a set of PID gains associated with it, and each should be tuned separately.

!!! tip
    For multirotors, the north and east configuration of the vehicle is usually symmetric, so the control gains for those two loops are likely to be very similar.

The `down_command_window` parameter saturates the input command for the down nominal (PID) controller to avoid large input commands (and thus large output commands).
In other words, if the input command is larger than `down_command_window`, then the input is saturated to this value.
This parameter exists to help stabilize the drone as it descends.

!!! warning "Important"
    The `down_command_window` parameter only applies on the descent, **not on the ascent**.
    This is because when the vehicle descends through the prop wash, the airflow can cause instability and shaking.


