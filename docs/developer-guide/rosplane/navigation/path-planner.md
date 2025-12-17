# Path Planner

## Overview
The path planner is responsible for creating, managing, and publishing waypoints.
In the current implementation, the path planner simply maintains a list of user-defined waypoints that the user can add to or clear.
The path planner then controls when waypoints are published to the path manager.

Another implementation of the path planner (as described in [the overview](./navigation-overview.md)) could include a visual-based path planner.
In this case, the path planner would receive sensor input and create and manage its own waypoints instead of a user-defined list of waypoints.
However, this new path planner would still be responsible for creating and publishing waypoints.
This ensures that the interface between the path planner and the path manager would stay the same, allowing for modularity in the path planner architecture.

## Interfaces
The path planner is implemented as a standalone ROS2 node, called `path_planner`.
This node subscribes to the topics it needs, provides services available to the user and other nodes, and publishes waypoints to the path manager, called `path_manager`.

The interface between `path_planner` and `path_manager` is the `/waypoint_path` topic,  which publishes messages with the `rosplane_msgs::msg::Waypoint` type.
This message contains information about the waypoint's location in NED (from the origin) or GNSS (LLA) coordinates, the desired airspeed at the waypoint, and the desired heading of the aircraft at the waypoint.

### About Waypoints
The following table contains the data members of the `rosplane_msgs::msg::Waypoint` objects and a brief description.

| **Member** | **Description** | **Type** | **Required** |
| :---: | :---: | :---: | :---: |
| `header` | Standard message header that contains a valid timestamp | `std_msgs::msg::Header` | Yes |
| `w` | Waypoint coordinates in NED or GNSS (LLA) | `std::array<float32, 3>` | Yes |
| `lla` | Flag to determine if the coordinates are passed in as NED or GNSS | `bool` | Yes |
| `chi_d` | Desired course at the waypoint | `double` | No |
| `use_chi` | Flag to use the `chi_d` value at the waypoint or not | `bool` | No - defaults to `false` |
| `va_d` | Desired airspeed at the waypoint | `double` | Yes |
| `clear_wp_list` | Clears all waypoints from `path_planner` and `path_manager` | `bool` | No |

#### Notes on the Waypoint object fields
The `w` field contains the coordinates of the waypoint in either NED coordinates (from the origin, defined as the place where ROSplane initialized) or GNSS coordinates given in LLA (latitude, longitude, and altitude).
The GNSS coordinates are converted to NED coordinates by `path_planner` before sending them to `path_manager`.

The `lla` field is a flag that tells `path_planner` if the coordinates provided are in the NED or world (GNSS) frame.
Set it to `true` if the coordinates for the waypoint are given in GNSS coordinates.
Note that GNSS and relative waypoints can be used together, meaning that one waypoint could be in the NED frame while the next one is in the global frame, since ROSplane converts global coordinates to the NED frame before publishing them to `path_manager`. 

!!! note
    Make sure that the `lla` field is set correctly if you decide to use GNSS coordinates, or your waypoints will be incorrect.

The `chi_d` field controls the desired course at the waypoint.
Note that this is only used if the `use_chi` flag is set to `true`. 

The `use_chi` field determines whether or not the `path_manager` should pay attention to the desired course (`chi_d`). 
If `use_chi` is set to `true` then `path_manager` will use a Dubins path framework to manage the waypoint.
If `use_chi` is set to `false`, then `path_manager` will ignore the desired course and will intead use straight lines and fillets to manage the transitions between waypoints.
See the [Path Manager](./path-manager.md) documentation for more information on this behavior.

The `clear_wp_list` is used internally by `path_planner` when the `/clear_waypoints` service is called.
It is recommended to use the service call instead of using this field manually.

!!! warning
    The `header`, `w`, `va_d`, and `lla` fields must be valid for every waypoint message sent.
    Failure to add these fields might crash the path planner or your aircraft.

### List of ROS2 Interfaces

| **ROS2 Interface** | **Topic or Service** | **Explanation** | **Message or Service Type** |
| :---: | :---: | :---: | :---: |
| `waypoint_publisher_` | `/waypoint_path` | Publishes the waypoints to the `path_manager` | `rosplane_msgs::msg::Waypoint` |
| `state_subscription_` | `/estimated_state` | Subscribes to the estimated state of the aircraft | `rosplane_msgs::msg::State` |
| `next_waypoint_service_` | `/publish_next_waypoint` | Publishes the next stored waypoint | `std_msgs::srv::Trigger` |
| `add_waypoint_service_` | `/add_waypoint` | Adds a new waypoint to list and optionally publishes it | `rosplane_msgs::srv::AddWaypoint` |
| `clear_waypoint_service_` | `/clear_waypoints` | Clears the list of waypoints from the path planner and publishes a clear waypoint command to the `path_manager` | `std_msgs::srv::Trigger` |
| `print_waypoints_service_` | `/print_waypoints` | Prints the saved waypoints to the terminal. Can be useful when debugging | `std_msgs::srv::Trigger` |
| `load_mission_service_` | `/load_mission_from_file` | Loads a mission from a YAML file | `rosflight_msgs::srv::ParamFile` |

## Parameters
See the [Parameter Management](../parameter-management.md) page for more details on how parameter management works.

### List of Parameters
| **Parameter** | **Explanation** | **Type** | **Range** |
| :---: | :---: | :---: | :---: |
| `num_waypoints_to_publish_at_start` | Number of waypoints to immediately publish at launch. If no waypoints are added, it will not publish any. | int | $\geq$ 0 |

## Recommended Usage
We recommend using a YAML file and the `/load_mission_from_file` service from the command line to load and fly waypoints.
See the example mission YAML file located at `/path/to/rosplane/rosplane/missions/fixedwing_mission.yaml`.

To call the service, run
```bash
ros2 service call /load_mission_from_file rosflight_msgs::srv::ParamFile "{filename: <FILENAME>}"
```
where `<FILENAME>` is the path to the mission YAML file.
