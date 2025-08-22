# ROScopter Overview

ROScopter is a basic multirotor autopilot built on ROS2 for use with the ROSflight flight controller.
It is built according to the methods published in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randy Beard and Dr. Tim McLain.

As per the [ROSflight vision](../../index.md#our-vision), ROScopter is *not* a fully-featured multirotor autopilot, but rather a simple, lean, ROS2-based autopilot.

The core ROScopter package is a simple waypoint-following autopilot.
This includes a navigation stack, a controller, and an estimator.
This can be seen in the figure below.

| ![Diagram of ROScopter architecture](../../assets/ROScopter-overview.svg "ROScopter architecture") |
|:--:|
|*Diagram of the ROScopter architecture*|

The structure of ROScopter and the nature of ROS2 interfaces allow ROScopter to be very modular, allowing you to write and integrate your own code without having to spend as much time working with interfaces and code integration. 
Since it is lean, the time to learn and understand the nature of ROScopter should be small compared to other, more featured (but more complex) autopilots.
This can improve research productivity, decrease debugging time, and improve the development of novel algorithms.

## Core Functionality

The ROScopter autopilot allows users to fly waypoint missions with an RC safety pilot.
The simplicity of this framework allows users to add their own autonomy stacks or mission requirements on top of the ROScopter stack.

<!-- For example, the `path_manager` node in the navigation stack in the core ROScopter package directs the `path_follower` node to follow either straight lines or circular arcs.
However, if a project needed to follow B-splines instead, the `path_manager` and the `path_follower` nodes could easily be replaced to achieve that.
Instead of loading a predetermined number of waypoints, higher levels of autonomy (i.e., vision-based guidance, etc.) can also be accomodated by building on top of the ROScopter stack by dynamically feeding the `path_planner` waypoints. -->

## Using ROScopter

For detailed instructions on how to use the core ROScopter package to fly autonomous waypoint missions, see the [ROScopter Setup](./roscopter-setup.md) page. 
This page can be used as a guide to building and running ROScopter before you start making your own changes to the autonomy stack.

For detailed instructions on each of the components of ROScopter, see the [ROScopter Developer Guide](../../developer-guide/roscopter/roscopter-dev-overview.md).
This page provides detailed instructions on how to use and change the code for each component of the ROScopter stack.
