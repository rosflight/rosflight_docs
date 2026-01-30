# Running Simulations with ROSflight

!!! tip "New to ROSflight?"

    If you are new to ROSflight, we recommend that you first start by setting up the simulation environment and learning to use the ROSflight ecosystem.
    Do this by following the [installation for sim](../installation/installation-sim.md) guides, and then the [ROSflight tutorials](../tutorials/index.md).

    The guides in this section assume you have already followed these tutorials.

ROSflight is a modular simulation package allowing it to perform software-in-the-loop (SIL) simulations of the ROSflight firmware.

!!! TODO
    add a good image of the sim here.

## Motivation
The goals of the ROSflight simulation module (called `rosflight_sim`) are to

- Enable **easy and extensive SIL** testing of an aircraft,
- Provide to the sim the **exact same software** that flies the physical aircraft,
- Support a **variety of simulators** out of the box -- from photorealistic to bare-bones, and
- Enable users to support their own simulator needs.

See the [simulator architecture](simulator-architecture.md) description page for more information on adding your own simulator to `rosflight_sim`.
This page also has more information on the modular structure of `rosflight_sim`.

The [detailed launching guide](./detailed-launching-guide.md) contains information on how to launch the simulation environment, more detailed than what is available in the [simulation tutorials](../tutorials/setting-up-rosflight-sim.md).

## A note on sims and viz
A _simulator_ includes many different modules, such as dynamic propagation, sensor creation, forces and moments computation, etc.
One of these modules is the _visualization_ module, that provides the graphical element to the simulator.
Different visualizers provide different functionality and require different information.
For example, [Gazebo Classic](https://classic.gazebosim.org/) handles the dynamic propagation (integration) for users, while a simple visualizer like RViz does not.

Since each _visualizer_ in large part determines what other functionality the _simulator_ modules need to provide, they are tightly coupled.
Thus, in this guide, we will refer interchangeably between _simulator_ and _visualizer_.

The `rosflight_sim` architecture has been designed to be as modular as possible in order to adapt as easily as possible to the different needs of various visualizers.
See the [simulation architecture page](./simulator-architecture.md) for more information.

## Sims that ship with `rosflight_sim`
The ROSflight simulation module was designed to be as modular as possible, in order to support different simulation and visualization needs.
Currently, we support 3 visualizers out-of-the-box:

- A "standalone" visualizer using [ROS2 RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html#rviz) tool (**recommended**)
- [Gazebo Classic](https://classic.gazebosim.org/)
- [HoloOcean](https://robots.et.byu.edu/holoocean) - a photorealistic simulator based on UE5.

Adding your own visualizer is part of what `rosflight_sim` was designed for.
See the [instructions on adding your own visualizer](./simulator-architecture.md#adding-your-own-visualizer) page for more information on plugging in your simulator into `rosflight_sim`.
