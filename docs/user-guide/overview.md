# Overview

!!! tip "New to ROS2?"

    If you are new to ROS2, we recommend you start with the [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html).
    We assume that users of ROSflight have a basic understanding of ROS2 CLI tools and interfaces.
    **We do not provide any ROS2 tutorials ourselves.**

!!! tip "New to ROSflight?"

    If you are new to ROSflight, we recommend that you first start by setting up the simulation environment and learning to use the ROSflight ecosystem.
    Do this by following the [installation for sim](./installation/installation-sim.md) guides, and then the [ROSflight tutorials](./tutorials/tutorial-overview.md).

    After you do that, you should be ready to start using ROSflight in your own research!
    We recommend you first read through the architecture documentation for [ROSplane](./concepts/rosplane-overview.md) and [ROScopter](./roscopter/roscopter-overview.md).

    Then visit the [Customizing ROSflight page](./customizing-rosflight.md) for more specifics on how to customize ROSflight for your research.

## Main Components of ROSflight

ROSflight is intended to be used with both a typical flight controller and a companion Linux computer.
Although it can be used with just a flight controller, this setup will not offer most of the advantages of ROSflight.

!!! note
    To avoid confusion, we try to consistently use the following terminology:

      - **Flight controller:** The embedded board that runs the ROSflight firmware and performs I/O with sensors and ESCs
      - **Companion computer:** A Linux computer, running ROS2, that is mounted on the vehicle and has a physical, serial connection with the flight controller
      - **Offboard control (setpoints):** The control setpoints passed from the companion computer to the flight controller. The control is "offboard" from the perspective of the flight controller, even though the computer providing those commands is mounted onboard the vehicle.

The following figure illustrates the interactions between the major components of ROSflight:

![System Components](images/components.svg)

### Firmware

The ROSflight [firmware](https://github.com/rosflight/rosflight_firmware) is the low level microcontroller code that runs on the flight controller. This communicates directly with sensors and actuators and serves as the bridge between hardware and higher level software.
The firmware itself is designed to do as little as possible, offloading most of the work to the companion computer.

Although higher level control is offloaded to the companion computer, enough control and functionality is included in the firmware to enable a safety pilot to fly the UAV through any portion of the flight with or without an operating companion computer.

### ROSflight IO

[ROSflight IO](https://github.com/rosflight/rosflight_ros_pkgs) is a ROS2 node that runs on the companion computer that communicates directly with the firmware over a serial connection. This serves as the bridge between the firmware and the rest of ROS2 network.

### ROSplane and ROScopter

[ROSplane](https://github.com/rosflight/rosplane) and [ROScopter](https://github.com/rosflight/roscopter) are ROS2 based autonomy stacks that run on the companion computer and do most of the heavy computation of the autopilot. Each portion of their autonomy stacks are organized into highly modular ROS nodes that can be easily swapped out with custom nodes.

ROSplane and ROScopter are not required for using ROSflight and you could choose to use an entirely different autonomy stack if you so desired.
See the [ROSplane](./concepts/rosplane-overview.md) and [ROScopter](./roscopter/roscopter-overview.md) documentation for a detailed description of the respective architectures and functionality.

### RC Safety Pilot

ROSflight is designed for use with offboard control from experimental and research code.
As such, it provides several mechanisms for an RC safety pilot to intervene if something goes wrong with the control setpoints coming from the companion computer.
See the [RC Setup](./concepts/rc-configuration.md) page for more information.

## Where do I start?

To get started with ROSflight, we recommend you first set up the simulation environment and walk through the tutorials to get familiar with the ROSflight ecosystem.

If you are ready to start with hardware experiments, first [install the required software](./installation/installation-hardware.md) and then follow the [hardware setup guide](./concepts/getting-started.md) to configure your vehicle for successful flight tests.
