# Path Manager

## Responsibility
The responsibility of the ROScopter `path_manager` is:

1. Take waypoints published by the [`path_planner`](./roscopter-path-planner.md) and produce a trajectory between waypoints, and
2. Monitor waypoint completion/switching to the next leg. 

## Interface with ROScopter
| ![Diagram of ROScopter architecture](../images/roscopter_architecture.svg) |
|:--:|
|*Reference diagram of the ROScopter architecture*|

The `path_manager` interfaces with the rest of ROScopter using publishers and subscribers.

| Subscriber name | Message type | Description |
| :--- | :--- | :--- |
| `estimated_state` | `roscopter_msgs/State` | Estimated state for the vehicle (needs position and orientation) |
| `waypoints` | `roscopter_msgs/Waypoint` | Waypoints to manage |

| Publisher name | Message type | Description |
| :--- | :--- | :--- |
| `trajectory_command` | `roscopter_msgs/TrajectoryCommand` | Output commanded trajectory |

To summarize, the `path_manager` takes in waypoints (from the `path_planner`) and the estimated state (from the `estimator`) and computes a desired trajectory, which it publishes on the `trajectory_command` topic.

## Using the `path_manager`/implementation details
!!! note

    In this section, we will refer to the path/trajectory between two waypoints as a "waypoint leg" of the mission

Given a set of waypoints and the position of the vehicle, the `path_manager` needs to compute a trajectory to get from one waypoint to the next.
We would like trajectory setpoints $u_\text{traj}$ of the form
$$
    u_\text{traj} = [p, \dot{p}, \ddot{p}, \psi, \dot{\psi}, \ddot{\psi}]^T,
$$
where $p$ is the 3-D position, and $\psi$ is the desired heading.
The velocity and acceleration parts are essentially feed-forward terms that can be used by the downstream [trajectory follower](./roscopter-trajectory-follower.md).

!!! note
    The following path derivation is very similar to the one in [*Small Unmanned Aircraft: Theory and Practice*](https://github.com/byu-magicc/mavsim_public), Ch. 14 by Beard and McLain.

In the default implementation of ROScopter, the path the `path_manager` produces is simply a linear interpolation between waypoints, parametrized by the path parameter $\sigma(\tau)$.
This results in a straight line in 3D space, as well as a straight "line" between desired headings at the waypoints.
Thus, the commanded position, $p^c$, and heading, $\psi^c$, setpoints at the current time is computed as
$$
    p^c = \sigma(\tau) p_i + (1 - \sigma(\tau)) p_{i-1}
$$
$$
    \psi^c = \sigma(\tau) \psi_i + (1 - \sigma(\tau)) \psi_{i-1}
$$
where $p_i$ is the waypoint at the end of the current leg, $p_{i-1}$ is the waypoint at the start of the waypoint leg, $\sigma(\tau) \in [0,1]$ is a monotonically increasing function, and $\tau = \frac{t}{T} \in [0,1]$.
Here, $t$ is the current time along the current waypoint leg, and $T$ is the total time taken to travel the waypoint leg.

We can then compute the commanded derivatives as
$$
    \dot{p}^c = \frac{1}{T}\dot{\sigma}(\tau)(p_i - p_{i-1})
$$
$$
    \dot{\psi}^c = \frac{1}{T}\dot{\sigma}(\tau)(\psi_i - \psi_{i-1})
$$
$$
    \ddot{p}^c = \frac{1}{T^2}\ddot{\sigma}(\tau)(p_i - p_{i-1})
$$
$$
    \ddot{\psi}^c = \frac{1}{T^2}\ddot{\sigma}(\tau)(\psi_i - \psi_{i-1})
$$
Note that the $\frac{1}{T}$ term shows up because $\tau = t/T$.

To compute the desired output trajectory, we then need to define $\sigma(\tau)$.
We have some freedom to choose how $\sigma(\tau)$ behaves.
If we let $\sigma(\tau) = \tau = \frac{t}{T}$, then $p^c$ would interpolate linearly in time from $p_{i-1}$ to $p_i$.
However, this would mean $\dot{p}^c$ (i.e. the velocity) would be constant, and the vehicle would have that same velocity at $p_i$, causing it to overshoot as it continued on to the next leg.
<!--TODO: This warrants a picture/gif.-->

Instead, we will define $\sigma(\tau)$ to be a 5th order [smoothstep function](https://en.wikipedia.org/wiki/Smoothstep), which smoothly interpolates time from $0$ to $1$.
The smoothstep function guarantees that the velocity and acceleration are zero at $\tau=0$ and $\tau=1$, resulting in a smooth transition between waypoint legs.

The 5th order smoothstep scaling function (and derivatives) are defined as
$$
 \sigma(\tau) = 6\tau^5 - 15\tau^4 + 10\tau^3
$$
$$
 \dot{\sigma}(\tau) = 30\tau^4 - 60\tau^3 + 30\tau^2
$$
$$
 \ddot{\sigma}(\tau) = 120\tau^3 - 180\tau^2 + 60\tau
$$

We can almost compute the desired trajectory setpoints.
The last piece to consider is how to pick $T$, the total time to travel a given waypoint path leg.
The smaller $T$ is, the faster the path parameter $\sigma(\tau)$ will transition, and the faster the vehicle will need to fly.
For good performance, we would like to pick a $T$ such that the vehicle can feasibly follow the path given some constraints.

One way to do this is to pick $T$ so that it is based off of some user-defined maximum velocity and acceleration parameters.
To do this, let's first find where the maximums of $\dot{\sigma}(\tau)$ and $\ddot{\sigma}(\tau)$ are by setting derivatives equal to zero and solving for $\tau$
$$
\frac{\partial}{\partial \tau}\dot{\sigma}(\tau) = \ddot{\sigma}(\tau) = 120\tau^3 - 180\tau^2 + 60\tau = 0 \implies \tau = \frac{1}{2} = 0.5
$$
$$
\frac{\partial}{\partial \tau}\ddot{\sigma}(\tau) = 360\tau^2 - 360\tau + 60 = 0 \implies \tau = \frac{3 - \sqrt{3}}{6} = 0.2113 
$$

Now let's compute the magnitude of the desired velocity and acceleration commands generated at these $\tau$ points.
$$
    \lVert \dot{p}^c(\tau = 0.5) \rVert = \frac{1}{T_v}\dot{\sigma}(0.5) \lVert p_i - p_{i-1} \rVert = v_\text{max}
$$
$$
    \lVert \ddot{p}^c(\tau = 0.2113) \rVert = \frac{1}{T^2_a}\ddot{\sigma}(0.2113) \lVert p_i - p_{i-1} \rVert = a_\text{max}
$$

Now we solve for $T_v$ and $T_a$ to get
$$
T_v = \dot{\sigma}(0.5) * \lVert p_i - p_{i-1} \rVert / v_\text{max}
$$
$$
T_a = \sqrt{\ddot{\sigma}(0.2113) * \lVert p_i - p_{i-1} \rVert / a_\text{max}}
$$
Note that both $v_\text{max}$ and $a_\text{max}$ are user-defined parameters.

We can then choose the biggest $T$ so that both constraints are satisfied:
$$
T = \max (T_v, T_a)
$$

!!! warning "Tuning $v_\text{max}$ and $a_\text{max}$"
    The two parameters $v_\text{max}$ and $a_\text{max}$ are highly vehicle-specific.
    The `path_manager` uses them to make sure that the commanded trajectory never produces velocity and acceleration commands higher than these parameters (as described above).

    If these parameters are set too high for downstream controllers to track, it will cause poor performance (especially when using a controller that uses the velocity and acceleration feed-forward commands in the trajectory command vector).

    For example, if these parameters are too high, then the path parameter $\sigma$ will propagate faster than downstream controllers are able to track.
    As the vehicle falls behind the commanded trajectory setpoint, the feed-forward terms (velocity and acceleration commands) will get out-of-sync with the current position of the vehicle along the trajectory.

    This is especially noticeable when the setpoint reaches the end of the waypoint path.
    The feed-forward terms are designed to slow the vehicle as it reaches the end of the waypoint path (this is the purpose of the smoothstep function).
    If the vehicle is far behind the final position when the path parameter $\sigma$ reaches it, then those feed-forward terms will cause the vehicle to slow down, even though the vehicle is still somewhere in the middle of the waypoint path.
    This results in visually poor performance.
    <!-- TODO: This is an ideal spot for a graphic/GIF that shows this behavior. -->

    The solution? Tune the $v_\text{max}$ and $a_\text{max}$ parameters appropriately.

We can now plug along and compute our desired trajectory commands at each leg.
To set up each leg,

1. Compute $T$
1. Set $t = 0$

For each waypoint leg in our path, we

1. Compute $\tau = t/T$
2. Compute $\sigma(\tau)$ and derivatives
3. Compute $p^c$, $\psi^c$ and derivatives
4. Publish trajectory command

When the vehicle arrives at the next leg, simply set up the next leg and continue.

Note that the `path_manager` publishes trajectory commands at the rate specified by the `path_update_frequency` ROS 2 parameter.

<!--TODO: Nice graphics and visuals of the path manager and trajectory follower would be nice here. Maybe that would be an interesting project for someone?-->

### Path Manager Services
The `path_manager` offers the following service servers:

| Service name | Interface type | Description |
| :--- | :--- | :--- |
| `clear_waypoints` | `std_srvs/Trigger` | Clears the waypoints internal to the `path_manager` |
| `print_waypoints` | `std_srvs/Trigger` | Prints all waypoints received to the terminal |


!!! note "Clearing waypoints"
    As described in the [`path_planner`](./roscopter-path-planner.md#clear-waypoints) section, when the `path_planner` publishes a message with the `clear_wp_list` set true, the `path_manager` internally clears its waypoints.
    This does the same thing as the `clear_waypoints` service call.

    Thus, there is **no need** to publish a waypoint with `clear_wp_list=true` and call the `clear_waypoints` service.

!!! danger

    To avoid long descriptions, we omitted the namespace from the service calls in the documentation.
    All of the `path_manager` service servers are namespaced by `/path_manager`.

    In other words, to run the `clear_waypoints` service, you would run
    ```bash
    ros2 service call /path_manager/clear_waypoints ...
    ```
    **not**
    ```bash
    ros2 service call /clear_waypoints ...
    ```

## Parameters and configuration

The parameters associated with the `path_planner` are

| Parameter name | Parameter type | Description |
| :--- | :--- | :--- |
| `default_altitude` | `double` | Altitude of the default waypoint (see note). Should be a positive number (not in NED frame)! |
| `hold_last` | `bool` | Determines if the `path_manager` will hold at the last waypoint or if it will cycle back to the first |
| `path_update_frequency` | `double` | Rate (in Hz) at which to publish tractory setpoints |
| `waypoint_tolerance` | `double` | Distance (in m) from the target waypoint at which the waypoint is considered complete |
| `do_linear_interpolation` | `bool` | Determines if a linear time scaling or a 5th order smoothstep time scaling will be used.  |
| `max_acceleration` | `double` | Max acceleration value in m/s$^2$ (used when generating trajectory) |
| `max_velocity` | `double` | Max velocity value in m/s (used when generating trajectory) |


When no waypoints are provided to the `path_manager`, it will create a default waypoint to send to downstream tasks.
This default waypoint is at the origin (i.e. where the vehicle was armed), with an altitude determined by the `default_altitude` parameter.

The `hold_last` parameter determines whether or not the `path_manager` will hold the vehicle at the last waypoint.
If this is set to `false`, then the `path_manager` will repeatedly cycle through the waypoints once it reaches the last one.

The `path_update_frequency` parameter determines how often the `path_manager` publishes the trajectory commands.
Setting this rate does **not** determine how fast the trajectory setpoints interpolate between waypoints (see the [implementation details](#using-the-path_managerimplementation-details)).
Settings this to a high value just increases the frequency commands are published.

The last two parameters, `max_acceleration` and `max_velocity` are user-defined parameters that are **highly vehicle-specific**.
These two parameters determine the total time a given waypoint leg will take.
The `max_acceleration` parameter limits the acceleration computed by the `path_manager`.
The `max_velocity` parameter limits the velocity computed by the `path_manager`.
See the [implementation details](#using-the-path_managerimplementation-details) for more information on how they are used/defined.


