# Simulator Architecture

## Architecture of the SIL Simulation

!!! danger "TODO"

    continue here... This page is still under construction. Check back soon!

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

## Adding your own visualizer

