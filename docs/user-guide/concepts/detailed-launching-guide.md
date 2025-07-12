# Detailed Launching Guide

!!! danger "TODO"
    Continue here ... The purpose of this document is to give a deeper look into what is happening when we launch and all the launching options.

Detailed launching instructions for the `rosflight_sim` module.
For a quick copy-paste instructions, see the [quick start guide](./running-simulations-with-rosflight.md).

!!! note

    To simulate a fixed-wing mav, just change all instances of `multirotor` in the steps below to `fixedwing`.

## A note on sims and viz
A _simulator_ includes many different modules, such as dynamic propagation, sensor creation, forces and moments computation, etc.
One of these modules is the _visualization_ module, that provides the graphical element to the simulator.
Different visualizers provide different functionality and require different information.
For example, [Gazebo Classic](https://classic.gazebosim.org/) handles the dynamic propagation (integration) for users, while a simple visualizer like RViz does not.

Since each _visualizer_ in large part determines what other functionality the _simulator_ modules need to provide, they are tightly coupled.
Thus, in this guide, we will refer interchangeably between _simulator_ and _visualizer_.

## Sims that ship with `rosflight_sim`
The ROSflight simulation module was designed to be as modular as possible, in order to support different simulation and visualization needs.
Currently, we support 2 visualizers out-of-the-box:

- A "standalone" visualizer using [ROS2 RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html#rviz) tool (**recommended**)
- [Gazebo Classic](https://classic.gazebosim.org/)

Adding your own visualizer is part of what `rosflight_sim` was designed for.
See the [instructions on adding your own visualizer](./simulator-architecture.md#adding-your-own-visualizer) page for more information on plugging in your simulator into `rosflight_sim`.

This following sections detail how to launch and debug these two simulators.
**This guide assumes you have already installed ROS2 and have cloned and build the `rosflight_ros_pkgs` repository, as detailed in the [software installation for sim guide](../installation/installation-sim.md).**


## Standalone Sim
The "standalone" sim is a simulator that uses [ROS2 RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html#rviz) to visualize aircraft motion.

### Launching instructions
- Set up rosflight with the [software installation for sim guide](../installation/installation-sim.md) guide, making sure to install the `-desktop` package of ROS2, not the `-ros-base`.

- Launch the standalone sim for ROSflight SIL:
```bash
ros2 launch rosflight_sim multirotor_standalone.launch.py
```

- The standalone sim should now be running! You should have the following `rqt_graph`:
![multirotor_launch_rqt_graph](../images/rqt_graph_multirotor_standalone_launch.png)

!!! Tip
    Run `rqt_graph` with `rqt_graph` the in a new terminal, assuming the `-desktop` version of ROS2 was installed.

### Explanation 
The launch file manages launching several nodes all at once, as shown in the `rqt_graph` image:

- `/rosflight_io`: Handles the communication between the companion computer and the flight controller
- `/rosflight_sil_manager`: Calls the firmware at the correct time interval
- `/standalone_sensors`: Simulates sensor measurements given the true state of the robot
- `/rc`: Simulates the RC safety pilot connection to the firmware
- `/sil_board`: Instantiation of the firmware
- `/standalone_dynamics`: Dynamics node for keeping track of the true robot state
- `/multirotor_forces_and_moments`: Computes aerodynamic forces and moments based on motor commands
- `/standalone_time_manager`: Only appears if `use_sim_time` is set true. See [launch arguments](#launch-arguments)
- `/rviz`: Visualizer
- `/standalone_viz_transcriber`: Manages publishing `rosflight_sim` information to RViz
- 3 transform listener nodes: Manage coordinate frame transformations to RViz

For more information on each of these nodes, see the [simulator architecture](./simulator-architecture.md) page.


## Gazebo Classic
!!! DANGER
    Gazebo Classic is officially EOL as of January 2025, and **does not work with ROS2 Jazzy**.
    If you are using ROS2 Jazzy, please only use the standalond sim.
    The following instructions assume you are using ROS2 Humble.

    We have not yet upgraded to Gazebo, which is not EOL and better.
    If you would like to help in this effort, please visit the [GitHub issue](https://github.com/rosflight/rosflight_ros_pkgs/issues/166) and let us know :)

### Launching Instructions 

!!!TODO
    Change the ROS2 setup instructions include details about Gazebo and skipping if you don't want it.

* Set up ROSflight by following the [installation for sim guide](../installation/installation-sim.md)
* Source the Gazebo Classic setup file if you haven't added it to `~/.bashrc`:
```bash
source /usr/share/gazebo/setup.sh
```
* Launch Gazebo Classic with the ROSflight SIL:
```bash 
ros2 launch rosflight_sim multirotor_gazebo.launch.py aircraft:=multirotor
```

* Gazebo Classic should now be running! You should have the following `rqt_graph`.

![multirotor_launch_rqt_graph](../images/rqt_graph_multirotor_gazebo_launch.png)

!!! Tip
    Run `rqt_graph` with the command `rqt_graph` in a new terminal, assuming the `-desktop` version of ROS2 was installed.


### Explanation 
The launch file manages launching several nodes all at once, as shown in the `rqt_graph` image:

- `/rosflight_io`: Handles the communication between the companion computer and the flight controller
- `/rosflight_sil_manager`: Calls the firmware at the correct time interval
- `/standalone_sensors`: Simulates sensor measurements given the true state of the robot
- `/rc`: Simulates the RC safety pilot connection to the firmware
- `/sil_board`: Instantiation of the firmware
- `/gazebo`: Visualizer
- `/multirotor/dynamics`: Dynamics plugin to Gazebo Classic, computes the aerodynamic forces and moments given motor commands
- `/dynamics`: Interface node between the dynamics plugin and the rest of the `rosflight_sim` modules

For more information on each of these nodes, see the [simulator architecture](./simulator-architecture.md) page.

## Launch arguments
!!! Tip
    Command line arguments to launch files can be previewed by appending `--show-args` to the launch call:
    ```bash
    ros2 launch rosflight_sim multirotor_gazebo.launch.py --show-args
    ```

There are several command line arguments you can pass to customize the behavior at runtime.
Here are some important ones:

- `aircraft`: Defaults to "skyhunter". This parameter controls which dynamics and parameter files get loaded. Make sure this is set to your correct airframe!
- `use_sim_time`: By default, set false. This parameter is a parameter of all nodes in ROS2. If set to true on launch, the node will create a subscription to the `/clock` topic, and will use that as the source of time for its timers.
For Gazebo Classic, it is recommended to **leave this as false**, since Gazebo Classic publishes a `/clock` topic at 10Hz, which is too slow for most modules. If using the standalone sim, this parameter will allow you to speed up, slow down, or pause time. See the [simulation architecture](./simulator-architecture.md) page for more information.
- `use_vimfly`: Node that changes the default RC behavior to use VimFly, a program that lets you use Vim commands to fly around in the sim!
Vim, of course, is recommended for everyone, but VimFly especially if you don't have access to RC transmitter connected over USB.
See the [joystick](#joysticks) section for more information on what joysticks are supported.

These command line arguments should be passed using the `<argument>:=<value>` syntax.


## Joysticks
ROSflight supports several types of transmitters or controllers that you can use to fly around in the sim as the RC safety pilot.
If one of the supported transmitters is connected via USB at launch time, then the sim will default to using that controller instead of the default, **which is no RC connection**.
See the [Hardware Setup](./hardware-setup.md) guide for more information on joysticks.

!!! note
    It is much easier to fly with a real transmitter than with an Xbox-type controller.
    FrSky Taranis QX7 transmitters, Radiomaster TX16s transmitters, and RealFlight controllers are also supported.
    Non-Xbox joysticks may have incorrect mappings.
    If your joystick does not work, and you write your own mapping, please contribute back your new joystick mapping!

If you want to fly around in the sim and you don't have access to a transmitter, we recommend using VimFly, which allows you to fly around in the sim with your keyboard.
To use VimFly, just add the `use_vimfly:=true` string to the end of the launch command.

!!! example
    To launch the multirotor sim using the standalone simulator with VimFly, run
    ```bash
    ros2 launch rosflight_sim multirotor_standalone.launch.py use_vimfly:=true
    ```

## After launching

Remember that the SIL tries its best to replicate hardware.
That means you have to calibrate and set parameters in the same way you do in hardware.
See the [Parameter Configuration](./parameter-configuration.md) pages in this documentation for instructions on how to perform all preflight configuration before the aircraft will arm.

You can also run 
```bash
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
```
to load all required parameters and perform initial calibrations for a quick simulation setup.

