# ROScopter Overview

!!! tip "Quick Summary"

    Ready to get fly your code with ROScopter? Start here:

    1. Follow [the ROSflight tutorials](../tutorials/index.md) to set up ROScopter fly waypoint missions in sim.
    2. Read through [the following ROScopter architecture pages](./roscopter-path-planner.md) to understand the responsibilities of each node.
    3. Find examples of how to customize ROScopter in the [Customizing ROSflight](../customizing-rosflight.md) page.

ROScopter is a basic multirotor autopilot built on ROS2 for use with the ROSflight flight controller.
It is built according to the methods published in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randy Beard and Dr. Tim McLain.

As per the [ROSflight vision](../../index.md#our-vision), ROScopter is *not* a fully-featured multirotor autopilot, but rather a simple, lean, ROS2-based autopilot.

The core ROScopter package is a simple waypoint-following autopilot.
This includes a navigation stack (path planner, path manager, and trajectory follower), a controller, and an estimator.
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

For example, the `path_planner` module in the ROScopter navigation stack is responsible for compiling high-level waypoints and sending them to the `path_manager`.
The `path_planner` by default just takes in user-defined waypoints.
Instead of loading these user-defined waypoints, higher levels of autonomy (i.e., vision-based guidance, etc.) could be accommodated by building on top of the ROScopter stack by dynamically feeding the `path_planner` waypoints.

See [the ROScopter autonomy stack documentation pages](./roscopter-path-planner.md) for a detailed description of the default functionality and description of each module.

## Using ROScopter as-is

ROScopter's default waypoint-following functionality may be useful to some users.
The [ROSflight tutorials](../tutorials/index.md) walk users through setting up ROSflight and ROScopter in sim, all the way through flying waypoint missions.
Follow those tutorials first to get a feel for the default ROScopter behavior and workflow before you start making your own changes to the autonomy stack.

A detailed description of the ROScopter autonomy stack and each module is found in [the ROScopter architecture pages](./roscopter-path-planner.md).

## Customizing ROScopter

ROScopter's default functionality may not be sufficient for many users.
Because of this, ROScopter has been designed to be understandable, modular, and customizable.

The [customizing ROSflight page](../customizing-rosflight.md) describes how ROScopter is meant to be modified to assist in your research.
The page also includes examples and scenarios where each node can be modified, removed, or combined to accomplish different tasks.

## ROScopter Architecture Overview

!!! warning

    The following guides assume at least basic knowledge of ROS 2.
    See the [ROS 2 tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) if you have questions about the terminology in the following sections.

As seen in [the figure](#roscopter-architecture-overview), the flow of information through ROScopter cascades from module to module, with the estimator module feeding required information to all modules.
Each box in the figure represents a separate module or ROS 2 node that interfaces over the ROS 2 network using publishers, subscribers, and service calls.

In the default configuration of ROScopter, users supply waypoints to the `path_planner`, which then sends waypoints to the `path_manager`, which sends trajectory commands to the `trajectory_follower`.
The `controller` receives control setpoints from the `trajectory_follower` and sends those commands to the [flight control unit (FCU)](../overview.md) via the `rosflight_io` node.

Each module is described in detail in the following sections.

!!! tip

    For any ROS 2 interface definition in the following sections (message/service definition, etc.), you can find the message definition using 
    ```bash
    ros2 interface show <interface type>
    ```
