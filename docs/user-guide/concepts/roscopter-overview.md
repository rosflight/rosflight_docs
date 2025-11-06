# ROScopter Overview

!!! tip "Quick Summary"

    Ready to get fly your code with ROScopter? Start here:

    1. Follow [the ROSflight tutorials](../tutorials/tutorial-overview.md) to set up ROScopter fly waypoint missions in sim.
    2. Read through [the ROScopter architecture page](./roscopter-architecture.md) to understand the responsibilities of each node.
    3. Find examples of how to customize ROScopter in the [Customizing ROScopter](./customizing-roscopter.md) page.

ROScopter is a basic multirotor autopilot built on ROS2 for use with the ROSflight flight controller.
It is built according to the methods published in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randy Beard and Dr. Tim McLain.

As per the [ROSflight vision](../../index.md#our-vision), ROScopter is *not* a fully-featured multirotor autopilot, but rather a simple, lean, ROS2-based autopilot.

The core ROScopter package is a simple waypoint-following autopilot.
This includes a navigation stack, a controller, and an estimator.
This can be seen in the figure below.

| ![Diagram of ROScopter architecture](../images/roscopter_architecture.svg) |
|:--:|
|*Diagram of the ROScopter architecture*|

The structure of ROScopter and the nature of ROS2 interfaces allow ROScopter to be modular, allowing you to write and integrate your own code without having to spend as much time working with interfaces and code integration. 
Since it is lean, the time to learn and understand the nature of ROScopter should be small compared to other, more featured (but more complex) autopilots.
This can improve research productivity, decrease debugging time, and improve the development of novel algorithms.

## Core Functionality

The ROScopter autopilot allows users to fly waypoint missions with an RC safety pilot.
These waypoints are defined by desired 3-D locations and a desired heading.
The simplicity of this framework allows users to add their own autonomy stacks or mission requirements on top of the ROScopter stack.
See [the ROScopter autonomy stack documentation](./roscopter-architecture.md) for a detailed description of the default functionality and description of each module.

For example, the `path_planner` module in the ROScopter navigation stack is responsible for compiling high-level waypoints and sending them to the `path_manager`.
The `path_planner` by default just takes in user-defined waypoints.
Instead of loading these user-defined waypoints, higher levels of autonomy (i.e., vision-based guidance, etc.) could be accommodated by building on top of the ROScopter stack by dynamically feeding the `path_planner` waypoints.

## Using ROScopter as-is

ROScopter's default waypoint-following functionality may be useful to some users.
The [ROSflight tutorials](../tutorials/tutorial-overview.md) walk users through setting up ROSflight and ROScopter in sim, all the way through flying waypoint missions.
Follow those tutorials first to get a feel for the default ROScopter behavior and workflow before you start making your own changes to the autonomy stack.

A detailed description of the ROScopter autonomy stack and each module is found in [the ROScopter architecture page](./roscopter-architecture.md).

## Customizing ROScopter

ROScopter's default functionality may not be sufficient for many users.
Because of this, ROScopter has been designed to be understandable, modular, and customizable.

The [Customizing ROScopter page](./customizing-roscopter.md) describes how ROScopter is meant to be modified to assist in your research.
The page also includes examples and scenarios where each node can be modified, removed, or combined to accomplish different tasks.
