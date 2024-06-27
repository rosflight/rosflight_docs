# Overview

!!! tldr
    Visit [Getting Started](./getting-started.md) to start setting up ROSflight.
    Then visit [ROSplane Overview](./rosplane-overview.md) (for fixedwing aircraft) or [ROScopter Overview](./roscopter-overview.md) (for multirotor aircraft) to set up your autonomy stack.

## Main Components of ROSflight

ROSflight is intended to be used with both a typical flight controller and a companion Linux computer. Although it can be used with just a flight controller, this setup will not offer most of the advantages of ROSflight.

!!! note
    To avoid confusion, we try to consistently use the following terminology:

      - **Flight controller:** The embedded board that runs the ROSflight firmware and performs I/O with sensors and ESCs
      - **Companion computer:** A Linux computer, running ROS2, that is mounted on the vehicle and has a physical, serial connection with the flight controller
      - **Offboard control (setpoints):** The control setpoints passed from the companion computer to the flight controller. The control is "offboard" from the perspective of the flight controller, even though the computer providing those commands is mounted onboard the vehicle.

The following figure illustrates the interactions between the major components of ROSflight:

![System Components](images/components.svg)

### Firmware

The ROSflight [firmware](https://github.com/rosflight/rosflight_firmware) is the low level microcontroller code that runs on the flight controller. This communicates directly with sensors and actuators and serves as the bridge between hardware and higher level software. The firmware itself is designed to do as little as possible, offloading most of the work to the companion computer.

Although higher level control is offloaded to the companion computer, enough control and functionality is included in the firmware to enable a safety pilot to fly the UAV through any portion of the flight with or without an operating companion computer.

### ROSflight IO

[ROSflight IO](https://github.com/rosflight/rosflight_ros_pkgs) is a ROS2 node that runs on the companion computer that communicates directly with the firmware over a serial connection. This serves as the bridge between the firmware and the rest of ROS2 network.

### ROSplane and ROScopter

[ROSplane](https://github.com/rosflight/rosplane) and [ROScopter](https://github.com/rosflight/roscopter) are ROS2 based autonomy stacks that run on the companion computer and do most of the heavy computation of the autopilot. Each portion of their autonomy stacks are organized into highly modular ROS nodes that can be easily swapped out with custom nodes.

ROSplane and ROScopter are not required for using ROSflight and you could choose to use an entirely different autonomy stack if you so desired.

### RC Safety Pilot

ROSflight is designed for use with offboard control from experimental and research code.
As such, it provides several mechanisms for an RC safety pilot to intervene if something goes wrong with the control setpoints coming from the companion computer.
See the [RC Setup](./rc-configuration.md) page for more information.

## Where do I start?

To get started with ROSflight, visit the [Getting Started](./getting-started.md) page.
This page will direct you on how to set up your hardware and flight controller to run ROSflight.

After you set up ROSflight, you should set up an autonomy stack to interact with ROSflight.
Visit the [ROSplane Setup](./rosplane-setup.md) page for fixedwing aircraft or the [ROScopter Setup](./roscopter-setup.md) page for multirotor aircraft.
You can also use your own autonomy stack as described in [Autonomous Flight](./autonomous-flight.md).