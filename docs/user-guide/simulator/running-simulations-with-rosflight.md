# Running Simulations with ROSflight

ROSflight a modular simulation package allowing it to perform software-in-the-loop (SIL) simulations of the ROSflight firmware.

!!! TODO
    add a good image of the sim here.

## Motivation
The goals of the ROSflight simulation module (called `rosflight_sim`) are to

- Enable **easy and extensive SIL** testing of an aircraft,
- Provide to the sim the **exact same software** that flies the physical aircraft,
- Support a **variety of simulators** out of the box -- from photorealistic to bare-bones, and
- Enable users to plug in their own simulators if needed.

See the [quick start](#quick-start) guide below or see the [detailed launching guide](detailed-launching-guide.md).

See the [simulator architecture](simulator-architecture.md) description page for more information on adding your own simulator to `rosflight_sim`.

## Quick-Start
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
    Continue on the detailed launching guide. Certainly put the architecture on a separate page. There you can run through how everything is working.
    Add a page in the developer guide explaining how to add your own sim interfaces / dynamics / etc. to the sim.








## Architecture of the SIL Simulation

!!! TODO
    Make a figure of the new architecture. Maybe make it be a GIF? We could replace the table below

To best mimic the hardware experience of ROSflight, the SIL plugin for Gazebo actually implements the firmware source code as a library.
We just implemented a different "board layer" which uses gazebo instead of hardware calls for things like `imu_read()` and `pwm_write()`.
Instead of a serial link over USB to the flight controller, we use a UDP connection bouncing off of localhost to communicate between `rosflight_io` and the firmware.
This means the interface to the SIL plugin is identical to that of hardware.
The `rosflight_io` node is the main gateway to the firmware in simulation, just as it is in hardware.

The following table summarizes the correlation between connections in hardware and simulation:

| Connection Type                         | Hardware     | Simulation                               |
|-----------------------------------------|--------------|------------------------------------------|
| Serial communications to `rosflight_io` | USB / UART   | UDP                                      |
| RC                                      | PPM Receiver | ROS2 `RC` topic (`rosflight_msgs/RCRaw`) |
| Motors                                  | PWM          | Gazebo Plugin                            |
| Sensors                                 | SPI/I2C      | Gazebo Plugin                            |

## Troubleshooting
### Installation and Building
#### It doesn't build.
- Ensure git submodules are checked out at:
    - rosflight_ros_pkgs/rosflight_firmware
    - rosflight_ros_pkgs/rosflight_firmware/lib/eigen

































