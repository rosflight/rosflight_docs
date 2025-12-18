# ROSplane Overview

ROSplane is a basic fixed-wing autopilot built around ROS2 for use with the ROSflight autopilot.
It is built according to the methods published in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randy Beard and Dr. Tim McLain.

As per the [ROSflight vision](../../index.md#our-vision), ROSplane is *not* a fully-featured fixed-wing autopilot.
Instead, ROSplane is a simple, lean, ROS2-based fixedwing autopilot

The core ROSplane package is a simple waypoint-following autopilot.
This includes a navigation stack, a controller, and an estimator.
This can be seen in the figure below.

| ![Diagram of ROSplane architecture](../../assets/ROSplane-overview.svg "ROSplane architecture") |
|:--:|
|*Diagram of the ROSplane architecture*|

The structure of ROSplane and the nature of ROS2 interfaces allow ROSplane to be very modular, allowing you to write and integrate your own code without having to spend as much time working with interfaces and code integration. 
Since it is lean, the time to learn and understand the nature of ROSplane should be small compared to other, more featured (but more complex) autopilots.
This can improve research productivity, decrease debugging time, and improve the development of novel algorithms.

## Core Functionality

The ROSplane autopilot allows users to fly waypoint missions with an RC safety pilot.
These waypoints are defined by a 3-D location and (optionally) a desired heading at that location.

See [the ROSplane autonomy stack documentation pages](../../developer-guide/rosplane/rosplane-dev-overview.md) for a detailed description of each module and its default functionality.

## Using ROSplane as-is
The simplicity of the ROSplane framework allows users to add their own autonomy stacks or mission requirements on top of the ROSplane stack.


## Customizing ROSplane

For example, the `path_manager` node in the navigation stack in the core ROSplane package directs the `path_follower` node to follow either straight lines or circular arcs.
However, if a project needed to follow B-splines instead, the `path_manager` and the `path_follower` nodes could easily be replaced to achieve that.
Instead of loading a predetermined number of waypoints, higher levels of autonomy (i.e., vision-based guidance, etc.) can also be accomodated by building on top of the ROSplane stack by dynamically feeding the `path_planner` waypoints.

## Using ROSplane

For detailed instructions on how to use the core ROSplane package to fly autonomous waypoint missions, see the [ROSplane Setup](./rosplane-setup.md) page. 
This page can be used as a guide to building and running ROSplane before you start making your own changes to the autonomy stack.

For detailed instructions on each of the components of ROSplane, see the [ROSplane Developer Guide](../../developer-guide/rosplane/rosplane-dev-overview.md).
This page provides detailed instructions on how to use and change the code for each component of the ROSplane stack.
