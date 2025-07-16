# Running Simulations with ROSflight

ROSflight is a modular simulation package allowing it to perform software-in-the-loop (SIL) simulations of the ROSflight firmware.

!!! TODO
    add a good image of the sim here.

## Motivation
The goals of the ROSflight simulation module (called `rosflight_sim`) are to

- Enable **easy and extensive SIL** testing of an aircraft,
- Provide to the sim the **exact same software** that flies the physical aircraft,
- Support a **variety of simulators** out of the box -- from photorealistic to bare-bones, and
- Enable users to plug in their own simulators if needed.

See the [simulator architecture](simulator-architecture.md) description page for more information on adding your own simulator to `rosflight_sim`.

!!! tip "New to ROSflight?"

    If you are new to ROSflight, we recommend that you first start by setting up the simulation environment and learning to use the ROSflight ecosystem.
    Do this by following the [installation for sim](../installation/installation-sim.md) guides, and then the [ROSflight tutorials](../tutorials/tutorial-overview.md).

    After you do that, you should be ready to start using ROSflight in your own research!
    Visit the [developer guide pages](../../developer-guide/contribution-guidelines.md) for more specifics on how to use ROSflight for your research.

