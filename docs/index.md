# Welcome to ROSflight

***The ROS2 updates for ROSflight are still under development. Please use with discretion.***

## What is ROSflight?

ROSflight is a lean and adaptable autopilot system designed from the ground up with researchers in mind. Its purpose is to enable researchers to quickly and easily try out new ideas with minimal effort. Some of ROSflight's key feature are:

- Lightweight, modular, and well documented code that is easy to understand and modify.
- Most of the autopilot exists on a Linux computer rather than a microcontroller, enabling easier development with increased capabilities.
- Seamless switching between simulation and hardware: no part of the autopilot knows if it is running in simulation or not.
- Built on a ROS2 framework, allowing easy integration with ROS2 based projects.

A ROSflight setup typically consists of three main components:

1. The [firmware](), running on a typical flight controller, like a Pixhawk. This communicates directly with sensors and actuators and serves as the bridge between hardware and higher level software. The firmware itself is designed to do as little as possible, offloading most of the work to the companion computer.
2. The [rosflight_io node](), running on a companion computer that communicates directly with the firmware over a serial connection. This serves as the bridge between the firmware and the rest of ROS2 network.
3. A ROS2 autonomy stack running on the companion computer and communicating with rosflight_io. This is where most of the higher level control occurs and can contain anything from basic attitude control to complex machine learning algorithms. This can be any ROS2 based autonomy stack, but we've provided [ROSplane]() and [ROScopter]() as a good starting point.

## Why ROSflight?

There are a lot of excellent autopilots out there with a lot of great firmware options. Why did we feel like the world needed ROSflight? Because in our experience none of the other available options satisfied our research needs. Existing autopilots commonly used by researchers, like PX4 and Ardupilot, have massive codebases under constant development that run primarily on microcontrollers.

This presents a number of problems. First, we are limited in our ability to understand and control everything occuring within the autonomy stack, making much of the autopilot an untouchable black box. Second, we are limited to the abilities of microcontrollers and can't take advantage of the more powerful hardware and software that can be found on Linux computers. Third, maintaining up-to-date support for software or hardware projects becomes a lot of work as things are frequently changing.

ROSflight is intented to fix these problems by being a lightweight, modular, and easily understandable codebase that offloads as much as possible to a ROS2 based framework running on a Linux computer.

## Our Vision

Perhaps more important than what we are trying to accomplish is what we are *not* trying to accomplish. ROSflight is not intended to be a fully-featured autopilot with all the same functions of other autopilots, but instead serve as a core foundation that can easily be adapted to any use case.

Therefore, one of our primary objectives is to avoid feature creep and remain *lean*. We hope that others will extend our code and build on top of it, and would love to hear about your successes. But for the most part, we will not be incorporating these new features back into the main project. Instead, we hope that ROSflight will remain a lean, core code base that will continue to serve as a launch pad for exciting new projects and applications.
