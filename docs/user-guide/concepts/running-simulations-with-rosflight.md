# Running Simulations with ROSflight

ROSflight is a modular simulation package allowing it to perform software-in-the-loop (SIL) simulations of the ROSflight firmware.

!!! TODO
    add a good image of the sim here.

## Motivation
The goals of the ROSflight simulation module (called `rosflight_sim`) are to

- Enable **easy and extensive SIL** testing of an aircraft,
- Provide to the sim the **exact same software** that flies the physical aircraft,
- Support a **variety of simulators** out of the box -- from photorealistic to bare-bones, and
- Enable users to plug in their own simulators if needed.

See the [simulator architecture](simulator-architecture.md) description page for more information on adding your own simulator to `rosflight_sim`.

## First time setup
If you are setting up the simulation and using ROSflight for the first time, we recommend you do the following:

1. Follow the instructions on the [ROS2 setup guide](../ros2-setup.md) to install ROS2 and the `rosflight_ros_pkgs` repository.
1. Follow the [detailed-launching-guide](./detailed-launching-guide.md) to launch the standalone simulator.
1. Read the [joystick and post-launch instructions](./detailed-launching-guide.md#joysticks) to know how to load parameters and arm the aircraft.
1. Follow the guide to set up [ROScopter](../roscopter-overview.md) or [ROSplane](../rosplane-overview.md) autonomy stack.
1. Have fun with the autopilot in the sim!

## Quick-Start

!!! important
    Make sure to follow the [ROS2 setup guide](../ros2-setup.md) before running these instructions.

### Installation
```bash
// Clone from GitHub
cd /path/to/rosflight_ws/src
git clone --recursive https://github.com/rosflight/rosflight_ros_pkgs.git

// Install dependencies
cd rosflight_ros_pkgs
sudo rosdep init
rosdep update
rosdep install --from-path . -y --ignore-src

// Build the workspace
cd /path/to/rosflight_ws
colcon build

// Source the workspace
source install/setup.bash

// Add the source to the .bashrc (optional)
echo "source /path/to/rosflight_ws/install/setup.bash" >> ~/.bashrc
```

### Launching
```bash
// Ensure the `rosflight_ws` workspace is sourced (see installation instructions above)

// Launch a fixedwing simulation with VimFly (or connected joystick -- see detailed instructions)
ros2 launch rosflight_sim fixedwing_standalone.launch.py use_vimfly:=true

// Launch a multirotor simulation with VimFly (or connected joystick -- see detailed instructions)
ros2 launch rosflight_sim multirotor_standalone.launch.py use_vimfly:=true
```

This quick-start guide will launch the `standalone_sim` visualization engine (using RViz).
For other out-of-the box supported visualizers, see the detailed launching instructions below.

If you run into errors or problems when launching, please see the detailed launching instructions.

!!! TODO
    Finish architecture on a separate page. There you can run through how everything is working.
    Add a page in the developer guide explaining how to add your own sim interfaces / dynamics / etc. to the sim.

