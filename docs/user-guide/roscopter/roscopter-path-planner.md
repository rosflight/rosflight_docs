# Path Planner

## Responsibility
The `path_planner`'s responsibility is to define and publish a planned path consisting of waypoints.
Each waypoint is defined by a 3D location and orientation.

Thus, the `path_planner` maintains the list of all waypoints and publishes a subset to downstream nodes.
Note that the `path_planner` **does not manage** the path, meaning it does not determine how to transition from one waypoint to another.
It is simply in charge of maintaining and publishing waypoints.

## Interface with ROScopter
| ![Diagram of ROScopter architecture](../images/roscopter_architecture.svg) |
|:--:|
|*Reference diagram of the ROScopter architecture*|

The `path_planner` interfaces with the rest of ROScopter using publishers and subscribers.

| Subscriber name | Message type | Description |
| :--- | :--- | :--- |
| `estimated_state` | `roscopter_msgs/State` | Estimated state for the vehicle (needs GPS location of initial position) |

| Publisher name | Message type | Description |
| :--- | :--- | :--- |
| `waypoints` | `roscopter_msgs/Waypoint` | Waypoints in the planned path |

The `path_planner` takes in user-defined waypoints and publishes those waypoints to the `path_manager` over the `waypoints` topic.

## Parameters and configuration
The parameters associated with the `path_planner` are

| Parameter name | Parameter type | Description |
| :--- | :--- | :--- |
| `num_waypoints_to_publish_at_start` | `int` | Number of waypoints from plan to initially publish |

The `num_waypoints_to_publish_at_start` parameter determines how many waypoints are automatically published when a path plan is loaded.

If a given plan has 10 waypoints but the `num_waypoints_to_publish_at_start` is set to 5, then only the first 5 waypoints will be published right away.
The other waypoints will then be published sequentially using the appropriate service call listed below.

## Using the `path_planner`/implementation details
In the default configuration of ROScopter, the `path_planner` expects waypoints to be loaded using ROS 2 service calls.
This can be done by another node or manually by the user.
In our testing of the default ROScopter behavior, we usually manually load these waypoints by hand.

A planned path can be modified using the following service calls:

| Service name | Interface type | Description |
| :--- | :--- | :--- |
| `/add_waypoint` | `roscopter_msgs/AddWaypoint` | Add a single waypoint to the planned path |
| `/clear_waypoints` | `std_srvs/Trigger` | Clear all waypoints from the planned path |
| `/load_mission_from_file` | `rosflight_msgs/ParamFile` | Clear all waypoints and load a new plan from a file |
| `/print_waypoints` | `std_srvs/Trigger` | Print all waypoints in the plan to the screen |
| `/publish_next_waypoint` | `std_srvs/Trigger` | Publish the next waypoint in the plan |

!!! danger

    To avoid long descriptions, we omitted the namespace from the service calls in the documentation.
    All of the `path_planner` service servers are namespaced by `/path_planner`.

    In other words, to run the `add_waypoint` service, you would run
    ```bash
    ros2 service call /path_planner/add_waypoint ...
    ```
    **not**
    ```bash
    ros2 service call /add_waypoint ...
    ```

### Add waypoint
The `add_waypoint` service adds a single waypoint to the path.
See the `roscopter_msgs/AddWaypoint` interface definition to see what information is contained in the service call.

### Clear waypoints
This service clears all waypoints from the planned path.

Under the hood, this service call publishes a new waypoint that sets the `clear_wp_list` field to true (see the `roscopter_msgs/msg/Waypoint` message definition).
It is the responsibility of any node subscribing to the waypoints to clear any internal list of waypoints.
For example, in order to function properly, both the `path_manager` and the ROScopter ground control station (GCS) maintain internal lists of what waypoints have been published.
These nodes need to clear the internal storage of the waypoints when the `path_planner` publishes a waypoint with the `clear_wp_list` field set true.

### Load mission from file
This service loads a mission file to the `path_planner`.
The mission file is a YAML file containing the waypoint definitions.

This service call also clears all existing waypoints first, so the new waypoints will not be appended to the old waypoints.
If you want to append waypoints instead of clearing, use the `add_waypoints` service.

### Print waypoints
It can be helpful during debugging to know what waypoints are currently loaded to the `path_planner`.
This service call will print the waypoints to the terminal (the terminal running the `path_planner`, not the one that calls this service).

Note that printing to the screen is automatically done when using the `load_mission_from_file` service.

### Publish next waypoint
The `publish_next_waypoint` service server is used to publish the next unpublished waypoint in the planned path.
Note that the first <`num_waypoints_to_publish_at_start`> number of waypoints are automatically published whenever the `path_planner` gets new waypoints.


