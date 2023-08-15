# Welcome to ROSflight

***The ROS2 updates for ROSflight/ROScopter/ROSplane are still under development. Please use with discretion.***

## What is ROSflight?

ROSflight is a lean and adaptable autopilot system designed from the ground up with researchers in mind. Its purpose is to enable researchers to quickly and easily try out new ideas with minimal effort. Some of ROSflight's key feature are:

- Well documented, highly adaptable, amd lean code base making understanding and modifying any portion of ROSflight relatively easy.
- Seamless switching between simulation and hardware. No portion of ROSflight knows if it is running in a simulation or not, the only difference is where the sensor data is coming from.
- Built on a ROS2 framework, allowing easy integration with ROS2.

A ROSflight setup typically consists of two main components:

1. The ROSflight [firmware](https://github.com/rosflight/firmware), running on a typical flight controller, like a Pixhawk. This communicates with sensors and actuators and serves as the bridge between hardware and higher level software. The firmware itself is designed to do as little as possible, offloading most of the control systems to companion computer.
2. The ROSflight [rosflight_io node](https://github.com/rosflight/rosflight2), running on a Linux companion computer that communicates directly with the firmware over serial or ethernet. This serves as the bridge between the firmware and the rest of ROS2 network and contains no control systems.

## Why ROSflight?

There are a lot of excellent autopilots out there with a lot of great firmware options. Why did we feel like the world needed ROSflight? Because in our experience none of the other available options satisfied our research needs. Specifically, we needed an autopilot that could stream sensor data at high rates, easily accept control setpoints from a companion computer, and accomplish all of this with a lean, easy-to-understand code base.

The other options that we tried were limited in bandwidth for streaming sensor data, and the APIs for sending control setpoints were confusing and difficult to implement. Perhaps most importantly, the code was sometimes so complex (feature-rich, but complicated) that it was difficult to figure out what the autopilot was actually doing. In talking to other researchers and industry members, we found that many people shared similar frustrations. So we decided to create and share the autopilot we wanted, hoping it will be useful to other people as well.

## What are ROScopter and ROSplane?

One of the primary goals of ROSflight was to have the firmware do as little as is necessary, keeping it simple and allowing other higher-level software running on the companion computer to do most of the work. ROScopter and ROSplane are two examples of that higher-level software, and have been developed to work seamlessly with ROSflight. Both are built on the ROS2 framework and are designed to be very easy to understand and modify, serving as a base library for higher-level autonomy that can be easily built on. They are also useful if you need higher-level autonomy but aren't interested in developing it yourself.

Although we provide these libraries for those who want them, ROSflight is fully independent and can be used with different autonomy stacks or no autonomy stack at all.

!!!Note
    When we refer to ROSflight in our documentation, we are referring to the firmware and the rosflight_io node, not ROScopter or ROSplane. These are treated as separate packages.

## Our Vision

Perhaps more important than what we are trying to accomplish is what we are *not* trying to accomplish. ROSflight is not intended to be a fully-featured autopilot with all the same functions of other autopilots, but instead provide the minimal functionality required to keep a multirotor or fixed-wing vehicle in the air, and to serve as the foundation for writing new code to perform these higher-level tasks. 

Therefore, one of our primary objectives is to avoid feature creep and remain *lean*. We hope that others will extend our code and build on top of it, and would love to hear about your successes. But for the most part, we will not be incorporating these new features back into the main project. Instead, we hope that ROSflight will remain a lean, core code base that will continue to serve as a launch pad for exciting new projects and applications.
