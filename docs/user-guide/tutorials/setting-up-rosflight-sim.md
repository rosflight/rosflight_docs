# Setting Up ROSflight Sim

The purpose of this tutorial is to walk users through launching the default ROSflight simulator.

## Prerequisites

* Install the ROSflight software (`rosflight_ros_pkgs`) by following the [Installation for sim](../installation/installation-sim.md) guide.
* Ensure you have the `-desktop` version of ROS2 installed, not the `-ros-base` version, as GUI tools are required for visualization.

## ROSflight Sim Overview

A _simulator_ includes many different modules, such as dynamic propagation, sensor creation, forces and moments computation, etc.
One of these modules is the _visualization_ module, that provides the graphical element to the simulator.

While ROSflight is designed to support different visualizers, we focus on the most basic one in these tutorials, the `standalone_sim`, which uses the [ROS2 RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html#rviz) tool.

The ROSflight simulator is organized as a collection of ROS2 nodes that each provide different functionality.
As we work through launching the sim, we'll look at some of those modules and discuss what they do.

## Simulation Architecture

The ROSflight standalone simulator consists of several key components:

- **SIL Manager**: Manages the execution and timing of the firmware
- **SIL Board**: Instantiates the ROSflight firmware in software
- **Dynamics**: Simulates aircraft physics and dynamics
- **Sensors**: Simulates IMU, barometer, and other sensor data
- **Forces and Moments**: Computes aerodynamic forces based on control inputs
- **RViz**: Provides 3D visualization of the aircraft and flight path

## Launching `standalone_sim`

!!! warning
    Make sure you installed the `-desktop` version of ROS2, not the `-ros-base` version, or the GUI tools will not work.

1. Source your workspace:

    First, ensure your ROS2 environment and ROSflight workspace are properly sourced:

    ```bash
    source /opt/ros/humble/setup.bash
    source /path/to/rosflight_ws/install/setup.bash
    ```

2. Launch the simulator:

    ROSflight provides launch files for different aircraft types. Choose the appropriate command based on your needs:

    #### Multirotor Simulation

    ```bash
    ros2 launch rosflight_sim multirotor_standalone.launch.py
    ```

    #### Fixed-Wing Simulation

    ```bash
    ros2 launch rosflight_sim fixedwing_standalone.launch.py
    ```

    #### With Keyboard Control (VimFly)

    For manual control using keyboard input, add the `use_vimfly:=true` parameter:

    ```bash
    # Multirotor with keyboard control
    ros2 launch rosflight_sim multirotor_standalone.launch.py use_vimfly:=true

    # Fixed-wing with keyboard control
    ros2 launch rosflight_sim fixedwing_standalone.launch.py use_vimfly:=true
    ```

## Understanding the Simulation Environment

Let's look at what just happened when we launched.

TODO: be more descriptive here. Also add images

### RViz Visualization

Once launched, RViz will open displaying:

- **3D Aircraft Model**: Visual representation of your aircraft
- **Coordinate Frames**: Shows the aircraft's orientation and position
- **Sensor Data**: IMU vectors and other sensor information
- **Flight Path**: Trace of the aircraft's trajectory

### Running Nodes

You can verify the simulation is running by checking the active nodes:

```bash
ros2 node list
```

You should see the following nodes:

- `/rosflight_sil_manager`
- `/sil_board`
- `/standalone_sensors`
- `/standalone_dynamics`
- `/multirotor_forces_and_moments` or `/fixedwing_forces_and_moments`
- `/rosflight_io`

Information about these nodes and what they do can be found in the [simulation architecture description](../concepts/simulator-architecture.md).

### Topics

View the available topics to see the data flow:

```bash
ros2 topic list
```

Key topics include:

- `/rc_raw` - RC commands
- `/imu/data` - IMU sensor data
- `/attitude` - Aircraft attitude estimated by the firmware's estimator
- `/command` - Control command inputs to the firmware controller (or mixer)

<!-- ## Configuration and Customization -->
<!---->
<!-- ### Parameter Files -->
<!---->
<!-- ROSflight sim uses YAML configuration files located in the `rosflight_sim/params/` directory: -->
<!---->
<!-- - `multirotor_dynamics.yaml` - Physical parameters (mass, inertia, motor specifications) -->
<!-- - `multirotor_firmware.yaml` - Firmware configuration (PID gains, mixer settings) -->
<!-- - `fixedwing_firmware.yaml` - Fixed-wing specific firmware parameters -->
<!-- - `standalone_sim_params.yaml` - Simulator-specific parameters -->
<!---->
<!-- ### Customizing Aircraft Parameters -->
<!---->
<!-- To modify aircraft characteristics, edit the appropriate YAML file: -->
<!---->
<!-- ```bash -->
<!-- # Edit multirotor parameters -->
<!-- nano /path/to/rosflight_ws/src/rosflight_ros_pkgs/rosflight_sim/params/multirotor_dynamics.yaml -->
<!-- ``` -->
<!---->
<!-- Common parameters to modify: -->
<!-- - `mass` - Aircraft mass in kg -->
<!-- - `Jxx`, `Jyy`, `Jzz` - Moments of inertia -->
<!-- - `prop_max` - Maximum propeller force -->
<!-- - `motor_time_constant` - Motor response time -->
<!---->
<!-- After making changes, rebuild your workspace: -->
<!---->
<!-- ```bash -->
<!-- cd /path/to/rosflight_ws -->
<!-- colcon build --packages-select rosflight_sim -->
<!-- source install/setup.bash -->
<!-- ``` -->

## Troubleshooting

### Common Issues

??? warning "RViz Not Opening"
    - Ensure you installed `ros-humble-desktop`, not `ros-humble-ros-base`
    - Check that you have a display environment (not running in headless mode)

??? warning "Simulation Crashes"
    - Check parameter files for syntax errors
    - Verify all dependencies are installed: `rosdep install --from-path . -y --ignore-src`

<!-- ### Debugging Commands -->
<!---->
<!-- Check node status: -->
<!-- ```bash -->
<!-- ros2 node info /rosflight_sil_manager -->
<!-- ``` -->
<!---->
<!-- Monitor topic data: -->
<!-- ```bash -->
<!-- ros2 topic echo /attitude -->
<!-- ``` -->
<!---->
<!-- View parameter values: -->
<!-- ```bash -->
<!-- ros2 param dump /sil_board -->
<!-- ``` -->

## Review

In this tutorial, you learned how to:

- Launch the ROSflight standalone simulator for multirotor and fixed-wing aircraft
- Discover some of the simulation architecture and key components
- Troubleshoot common issues

## Next Steps

Once you have the simulator running, you can:

1. **[Firmware configuration and manual flight](./manually-flying-rosflight-sim.md)**: Configure the firmware with the necessary parameters and fly in sim with a supported controller
2. **[Autonomous flight](./setting-up-roscopter-in-sim.md)**: Integrate with the ROScopter or ROSplane autonomy stacks
3. **[Custom applications](../../developer-guide/contribution-guidelines.md)**: Use your own ROS2 nodes with ROSflight
4. **[Parameter/Gain tuning](./tuning-performance-in-sim.md)**: Use the RQT plugins to tune PID controllers and other parameters


