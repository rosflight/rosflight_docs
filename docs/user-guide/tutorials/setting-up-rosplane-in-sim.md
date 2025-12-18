# Setting up ROSplane in Sim

This tutorial guides you through setting up ROSplane, the fixed-wing autopilot system, in simulation.
ROSplane provides autonomous flight capabilities for fixed-wing aircraft.

This tutorial will walk you through:

- Launching the `rosplane` autonomy stack
- Flying waypoint missions
- Some basic analysis of what is going on

## Prerequisites

- [Manually flying in ROSflight Sim](./manually-flying-rosflight-sim.md)

## ROSplane Overview

ROSplane is a ROS2-based autopilot system designed for fixed-wing vehicles.

### Major System Components

- **Estimator**: EKF for state estimation from IMU, GPS, and barometer data
- **Controller**: Multiple control algorithms (successive loop control, total energy control)
- **Path Follower**: Tracks commanded paths and generates control setpoints
- **Path Manager**: Converts waypoints into smooth flyable paths using fillets or Dubins curves
- **Path Planner**: High-level mission planning and waypoint management

### Control Hierarchy

ROSplane implements a hierarchical control structure where high-level waypoint commands flow through path planning, path management, path following, and finally to low-level control loops.

## Launching `standalone_sim`

The standalone simulator provides a lightweight simulation environment using RViz for visualization.
This is the recommended starting point for ROSplane simulation.

Launch the fixed-wing simulation:

```bash
# Start the standalone simulator with ROSflight firmware simulation
cd ~/rosflight_ws
ros2 launch rosflight_sim fixedwing_standalone.launch.py use_vimfly:=true
```

The RViz simulation environment should launch.

See the [manually flying guide](./manually-flying-rosflight-sim.md) for instructions on launching, configuring, and arming in sim.

## Launching ROSplane Autonomy Stack

The ROSplane autonomy stack is a collection of ROS2 nodes that provide autonomous flight capabilities on top of the basic simulation.

In a new terminal, run:

```bash
ros2 launch rosplane_sim sim.launch.py
```

This launch file does 2 things:

1. Launches `rosplane` autonomy stack by calling the `rosplane.launch.py` file.
This file launches most of the nodes we'll explore later.
2. Starts the `sim_state_transcriber` node.
This node publishes the truth state from the simulation as a `rosplane/msg/State` message so we can easily compare estimated and true state.

### Understanding the ROSplane Stack

Let's take a look at the nodes that we just ran:

```bash
# Check ROSplane-specific nodes
ros2 node list
```

You should see the following output:
```bash
➜  ~ ros2 node list
/autopilot
/estimator
/path_follower
/path_manager
/path_planner
/rosplane_truth
```

??? info "**Node Descriptions**"
    - **`/autopilot`**: Main control node that implements multiple control algorithms (successive loop control, total energy control) and manages flight modes
    - **`/estimator`**: EKF that fuses IMU, GPS, and barometer data to provide state estimation (position, velocity, attitude, airspeed)
    - **`/path_follower`**: Tracks commanded paths and generates control setpoints for the controller
    - **`/path_manager`**: Converts waypoints into smooth flyable paths using fillets or Dubins curves
    - **`/path_planner`**: High-level mission planning node that handles waypoint loading and mission sequencing
    - **`/rosplane_truth`**: Simulation truth state publisher that provides ground truth data for comparison with estimated state

Let's now take a look at the topics specific to `rosplane`:

```bash
# Check ROSplane-specific topics
ros2 topic list
```

You should see the following output (note that these are topics from only the `rosplane` launch file):
```bash
➜  ~ ros2 topic list
/airspeed
/baro
/command
/controller_command
/controller_internals
/current_path
/estimated_state
/gnss
/imu/data
/parameter_events
/rosout
/sim/rosplane/state
/sim/truth_state
/sim/truth_wind
/status
/waypoint_path
```

??? info "**Some Key Topic Descriptions**"
    - Messages that the `estimator` subscribes to:
        - **`/airspeed`**: Differential pressure sensor for airspeed estimation
        - **`/baro`**: Barometric pressure sensor data for altitude estimation
        - **`/gnss`**: GPS position and velocity measurements
        - **`/imu/data`**: IMU sensor data (accelerometer, gyroscope measurements)
        - **`/magnetometer`**: Magnetometer readings for heading estimation
    - **`/command`**: Commands sent to ROSflight firmware (see rosflight_msgs/msg/Command for details)
    - **`/controller_internals`**: Inner control loop commands. Helpful for debugging
    - **`/current_path`**: Active path segment being followed by the path follower
    - **`/estimated_state`**: Complete vehicle state from EKF (position, velocity, attitude, airspeed, wind estimation)
    - **`/sim/rosplane/state`**: ROSplane-formatted state message from simulation truth
    - **`/waypoint_path`**: Current mission waypoints and path information

When we fly waypoint missions, we will load waypoints to the `path_planner` using a service call.
The chain of information flows from the `path_planner` to the `path_manager`, `path_follower`, `autopilot`, and finally on to the firmware.

### Launch Ground Control Station

The ground control station will plot waypoints that we pass to ROSplane.
It can be helpful to launch this so we can see if ROSplane is actually doing what we want it to do.

```bash
# In a new terminal (source workspace first)
ros2 launch rosplane_gcs rosplane_gcs.launch.py
```

This will launch another instance of RViz that will display different information than the main simulation pane.

## Loading Missions

ROSplane supports loading waypoint missions through waypoints defined in YAML files or set through ROS services.
These waypoints will be uploaded to the `path_planner` node using the `path_planner`'s ROS2 services.

### Using Waypoint Files

Waypoints can be loaded in batch manner from a file.

Create or modify waypoint files:
```bash
# Edit the default waypoint file
vim ~/rosflight_ws/src/rosplane/rosplane/missions/fixedwing_mission.yaml
```

Example waypoint file structure:
```yaml
# WAYPOINTS
wp:
  w: [1000.0, -500.0, -50.0]  # Position [North, East, Down] in meters
  chi_d: 1.1518               # Desired heading (radians)
  lla: false                  # Use NED coordinates (not GPS lat/lon/alt)
  use_chi: false              # Use smooth turns
  va_d: 15.0                  # Desired airspeed (m/s)
wp:
  w: [1000.0, 400.0, -50.0]
  chi_d: 1.1518
  lla: false
  use_chi: false
  va_d: 15.0
```

**Waypoint Parameters:**

- **`w`**: Position coordinates `[North, East, Down]` (in meters NED frame or LLA)
- **`chi_d`**: Desired heading in radians
- **`lla`**: Set to `false` for NED coordinates, `true` for GPS coordinates
- **`use_chi`**: Set to `false` for fillet path planning, `true` for Dubin's path planning. See _Small Unmanned Aircraft: Theory and Practice_ for more information.
- **`va_d`**: Desired airspeed in m/s

Load waypoints from mission file using the service call:

```bash
# Load waypoints from the default mission file
cd ~/rosflight_ws/src/rosplane/rosplane/params
ros2 service call /load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/fixedwing_mission.yaml}"
```

### Setting Waypoints Manually

You can also add waypoints dynamically using the following services:

```bash
# Add a single waypoint (N=500m, E=300m, D=-40m, heading=0rad, airspeed=12m/s)
ros2 service call /add_waypoint rosplane_msgs/srv/AddWaypoint \
  "{w: [500.0, 300.0, -40.0], chi_d: 0.0, use_chi: false, va_d: 12.0}"

# Progress to next waypoint in sequence
ros2 service call /publish_next_waypoint std_srvs/srv/Trigger

# Print current waypoint list
ros2 service call /print_waypoints std_srvs/srv/Trigger
```

### Verify Mission Loading

You can check that the waypoints are loaded by looking at the `rosplane_gcs` RViz GUI.
You should see the waypoints plotted as markers in the visualization.

### Publishing Additional Waypoints

Note that the `path_planner` will publish only the first few waypoints (determined by the `num_waypoints_to_publish_at_start` parameter).
Publish the next one by calling:
```bash
ros2 service call /publish_next_waypoint std_srvs/srv/Trigger
```
or by setting the parameter to the desired value:
```bash
ros2 param set /path_planner num_waypoints_to_publish_at_start 100
```

## Enabling Autonomous Flight

After loading missions, enable autonomous flight through `rc`'s services.
When starting up ROSflight, RC override will be **enabled** by default, meaning that the companion computer will not control the vehicle.
To enable offboard control, you will need to **disable RC override**.

### Arm and Start Mission

!!! note

    If using VimFly or a transmitter, these services will not be available.
    Use the transmitter or VimFly to arm and disable RC override.

```bash
# Arm the vehicle (enable motors)
ros2 service call /toggle_arm std_srvs/srv/Trigger

# Turn off RC override -- make sure it is toggled on before arming
ros2 service call /toggle_override std_srvs/srv/Trigger
```

### Monitor Flight Progress

In ROScopter, every module communicates with the other modules via ROS2 publishers and subscribers.
You can track the status and state of the vehicle by echoing the relevant ROS2 topics.
We often will use [PlotJuggler](./tuning-performance-in-sim.md#install-and-launch-plotjuggler) to visualize the data to monitor what is going on internal to the system.

For example:
```bash
# Monitor vehicle state during flight
ros2 topic echo /estimated_state

# Watch controller commands internal to ROSplane
ros2 topic echo /controller_internals

# Watch controller commands sent to ROSflight firmware
ros2 topic echo /command
```

### Tuning Flight Performance

It is possible that the flight performance is unstable due to the `controller`'s gains not being set correctly.
See [the tuning guide](./tuning-performance-in-sim.md) for more information.

## Helpful Tips

### Resetting the Simulation State

The `standalone_sim` module has a very simplistic ground plane representation (i.e. no friction).
This means that when the aircraft is armed, it will drift over time.
If this happens to you, you can reset the simulation state of the vehicle.

You can reset the state of the vehicle by using a service call provided by the `dynamics` node using
```bash
ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState
```

Note that in this command we didn't specify what the values for the `SetSimState` message type are, so they default to zero.
This should move the vehicle to the origin in the standalone sim.
If you are using the Gazebo simulator, a position of \[0,0,0\] is underground, so the vehicle will respond erratically.

You can set the simulation state to any arbitrary state by providing the information in the `SetSimState` service definition.

??? tip "How do I know what information is contained in a message definition?"

    One way is to go find the message definition file in the package where it was built (i.e. rosflight_msgs package in the `rosflight_ros_pkgs` repository).

    Another option is to use the `ros2 interface show <name-of-interface>` command.
    For example, to find out what is included in the `SetSimState` service definition, do
    ```bash
    ros2 service call rosflight_msgs/srv/SetSimState
    ```
    and look through the output.

## Review

You have successfully completed the ROSplane autonomous flight tutorial. You should now be able to:

- **Launch ROSplane Stack**: Start the complete autonomy stack with estimator, controller, and path management
- **Load Waypoint Missions**: Create and load waypoint missions from YAML files or via ROS services
- **Execute Autonomous Flight**: Arm the vehicle and fly autonomous waypoint missions
- **Monitor Flight Performance**: Track vehicle state and controller performance during flight
- **Understand System Architecture**: Recognize how nodes communicate and data flows through the system

## Next Steps

Once you have ROSplane running autonomously, you can:

1. **[Multirotor Autonomous Flight](./setting-up-roscopter-in-sim.md)**: Explore ROScopter for multirotor autonomous flight
1. **[Parameter/Gain Tuning](./tuning-performance-in-sim.md)**: Use RQT plugins to tune PID controllers and optimize flight performance
1. **[Custom Applications](../../developer-guide/contribution-guidelines.md)**: Develop your own ROS2 nodes that interface with ROSplane

### Additional Resources

- [ROSflight Parameter Reference](../concepts/parameter-configuration.md): Detailed firmware parameter descriptions
- [Hardware Setup Guide](../concepts/hardware-setup.md): Preparing real hardware for flight
- [ROSplane Architecture Documentation](../../developer-guide/rosplane/rosplane-dev-overview.md): In-depth system design and implementation details

