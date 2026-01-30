# Detailed Launching Guide

Detailed launching instructions for the `rosflight_sim` module.
For a quick tutorial on running the `standalone_sim`, see the [simulator tutorials](../tutorials/setting-up-rosflight-sim.md).



## Joysticks
ROSflight supports several types of transmitters or controllers that you can use to fly around in the sim as the RC safety pilot.
If one of the supported transmitters is connected via USB at launch time, then the sim will default to using that controller instead of the default, **which is no RC connection**.
See the [Hardware Setup](../hardware-and-rosflight/hardware-setup.md#joystick) guide for more information on joysticks.

!!! note
    It is much easier to fly with a real transmitter than with an Xbox-type controller.
    FrSky Taranis QX7 transmitters, Radiomaster TX16s transmitters, and RealFlight controllers are also supported.
    Non-Xbox joysticks may have incorrect mappings.
    If your joystick does not work, and you write your own mapping, please contribute back your new joystick mapping!

If you want to fly around in the sim and you don't have access to a transmitter, we recommend using VimFly, which allows you to fly around in the sim with your keyboard.
To use VimFly, just add the `use_vimfly:=true` string to the end of the launch command.

!!! example
    To launch the multirotor sim using the standalone simulator with VimFly, run
    ```bash
    ros2 launch rosflight_sim multirotor_standalone.launch.py use_vimfly:=true
    ```

## After launching

Remember that the SIL tries its best to replicate hardware.
That means you have to calibrate and set parameters in the same way you do in hardware.
If you need a reminder, please follow the [configuration and manual flight tutorial](../tutorials/manually-flying-rosflight-sim.md).

See the [Parameter Configuration](../rosflight-firmware/parameter-configuration.md) pages in this documentation for instructions on how to perform all preflight configuration before the aircraft will arm.

You can also run 
```bash
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
```
to load all required parameters and perform initial calibrations for a quick simulation setup.

!!! warning

    Remember to verify that all parameters are set to the value that you would expect.
    Wrong parameters is a **common source of error** in sim and in hardware.
