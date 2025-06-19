# Detailed Launching Guide
Detailed launching instructions for the `rosflight_sim` module.
For a quick copy-paste instructions, see the [quick start guide](running-simulations-with-rosflight.md#quick-start)

!!! note
    To simulate a fixed-wing mav, just change all instances of `multirotor` in the steps below to `fixedwing`.

## A note on sims and viz
A _simulator_ includes many different modules, such as dynamic propagation, sensor creation, forces and moments computation, etc.
One of these modules is the _visualization_ module, that provides the graphical element to the simulator.
Different visualizers provide different functionality and require different information.
For example, [Gazebo Classic](https://classic.gazebosim.org/) handles the dynamic propagation (integration) for users, while a simple visualizer like RViz does not.

Since each _visualizer_ in large part determines what other functionality the _simulator_ modules need to provide, they are tightly coupled.
Thus, in this guide, we will refer interchangeably between _simulator_ and _visualizer_.

## Sims that ship with `rosflight_sim`
The ROSflight simulation module was designed to be as modular as possible, in order to support different simulation and visualization needs.
Currently, we support 2 visualizers out-of-the-box:

- [Gazebo Classic](https://classic.gazebosim.org/)
- A "standalone" visualizer using [ROS2 RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html#rviz) tool

Adding your own visualizer is part of what `rosflight_sim` was designed for.
See the [instructions on adding your own visualizer](simulator-architecture.md#adding-your-own-visualizer) page for more information on plugging in your simulator into `rosflight_sim`.

This following sections detail how to launch and debug these two simulators.

!!! TODO
  Continue here with the detailed launching guide. Add figures and the architecture image to show what nodes run with Gazebo

## Gazebo Classic

* Setup ROSflight with the [ROS2 Setup](ros2-setup.md) guide, making sure to install the `-desktop` package of ROS2, not the `-ros-base`.

* Source the Gazebo setup file if you haven't added it to `~/.bashrc`:
```bash
source /usr/share/gazebo/setup.sh
```

* Launch Gazebo with the ROSflight SIL:
```bash 
ros2 launch rosflight_sim multirotor.launch.py
```

* Gazebo should now be running, and you should have the following `rqt_graph`.

![multirotor_launch_rqt_graph](images/rqt_graph_multirotor_launch.png)

* At this point, you can't actually do anything because there is no RC connection and no `rosflight_io` to talk to the firmware. Let's start by running a `rosflight_io` node. In a separate terminal, run:
```bash
ros2 run rosflight_io rosflight_io --ros-args -p udp:=true
```

    * The `udp` parameter tells `rosflight_io` to simulate a serial connection over UDP rather than using the USB connection to hardware

Your `rqt_graph` should look something like the following image. This looks funny because ROS2 doesn't actually know that there is a UDP connection between `rosflight_io` and gazebo. There is one, though, and you can test it by echoing any of the topics published by `rosflight_io`.

![rqt_graph_multirotor_launch_with_rosflight_io](images/rqt_graph_multirotor_launch_with_rosflight_io.png)

* Start up a simulated RC connection. The easiest way to do this is with the ROSflight utility `rc_joy.py`. Connect a joystick to the computer (or transmitter) and run: 
    ```bash
    ros2 run rosflight_sim rc_joy.py --ros-args --remap /RC:=/multirotor/RC
    ```
    This simulates the RC connection in hardware. If everything is mapped correctly, you should now be able to arm, disarm and fly the aircraft in simulation!

!!! tip
    To start the Gazebo sim, rosflight_io node, and rc_joy.py utility all at once, run this command instead of the three commands individually:
    ```bash
    ros2 launch rosflight_sim multirotor_sim_io_joy.launch.py    
    ```

!!! note
    It is much easier to fly with a real transmitter than with an Xbox-type controller. FrSky Taranis QX7 transmitters, Radiomaster TX16s transmitters, and RealFlight controllers are also supported. Non-Xbox joysticks may have incorrect mappings. If your joystick does not work, and you write your own mapping, please contribute back your new joystick mapping!

Remember, the SIL tries its best to replicate hardware. That means you have to calibrate and set parameters in the same way you do in hardware. See the [Hardware Setup](hardware-setup.md) and [Parameter Configuration](parameter-configuration.md) pages in this documentation for instructions on how to perform all preflight configuration before the aircraft will arm. You can also run 
```bash
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
```
to load all required parameters and perform initial calibrations for a quick simulation setup.

