# Setting ROSflight Sim

The purpose of this tutorial is to walk users through launching the default ROSflight simulator.

#### Prerequisite Tutorials and guides:
* Install the ROSflight software (`rosflight_ros_pkgs`) by following the [Installation for sim](./setting-up-rosflight-sim.md) guide.

## ROSflight Sim Overview
A _simulator_ includes many different modules, such as dynamic propagation, sensor creation, forces and moments computation, etc.
One of these modules is the _visualization_ module, that provides the graphical element to the simulator.

While ROSflight is designed to support different visualizers, we focus on the most basic one in these tutorials, the `standalone_sim`, which uses the [ROS2 RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html#rviz) tool.

The ROSflight simulator is organized as a collection of ROS2 nodes that each provide different functionality.
As we work through launching the sim, we'll look at some of those modules and discuss what they do.
Please see the [simulation architecture concept page](../concepts/simulation-architecture.md) for more information.

## Launching `standalone_sim`
!!! warning
    Make sure you installed the `-desktop` version of ROS2, not the `-ros-base` version, or the GUI tools will not work.

TODO: Continue here with the documentation. Move the launching files over to here, and continue with the tutorials.



TODO: Link to a concept page where we talk about the tools we like to use for introspection. Plotjuggler, RQT, etc.

## Review

