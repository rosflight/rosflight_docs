# Manually flying in ROSflight Sim

The purpose of this tutorial is to enable users to **configure the ROSflight firmware** through the parameter interface and **fly in sim** using a keyboard or a supported transmitter.

## Prerequisites:
* [Setting up ROSflight sim](./setting-up-rosflight-sim.md)

## Overview

This tutorial walks through the complete process of manually flying an aircraft in ROSflight simulation. You will learn to:

- Load firmware parameters appropriate for your aircraft
- Calibrate the IMU
- Saving parameters to memory
- Fly using a supported RC transmitter or keyboard controls via VimFly
- Troubleshoot common issues during manual flight

By the end of this tutorial, you will have a fully configured simulation environment ready for manual flight operations.

## Loading firmware parameters

After launching the simulator, you must load the firmware parameters.
Parameters control the behavior of the firmware and how users will interact with the firmware.
The ROSflight is highly configurable, but we have provided YAML files that contain a good default configuration for both multirotors and fixedwing vehicles.

Parameter configuration is handled by the `rosflight_io` node, which exposes some of the firmware's parameters to the ROS2 parameter system.
In addition, `rosflight_io` has many services that allow users to configure the firmware.
Ensure `rosflight_io` (and the rest of the simulation is running) using
```bash
ros2 node list
```
and verifying that the `rosflight_io` and `sil_board` nodes are included in the list.

### Loading parameters manually
You can load parameters one-by-one or with a YAML file, as described in the [parameter configuration guide](../concepts/parameter-configuration.md).
We will load parameters from a file.

1. Navigate to the params directory:
  ```bash
  cd /path/to/rosflight_ws/src/rosflight_ros_pkgs/rosflight_sim/params
  ```

1. Load the multirotor or fixedwing parameter YAML files:
  ```bash
  # For multirotor
  ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename: $(pwd)/multirotor_firmware/multirotor_combined.yaml}"

  # For fixedwing
  ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename: $(pwd)/fixedwing_firmware.yaml}"
  ```

Note that we first navigated to the directory so we could use the built-in `pwd` Linux command.
It just saves time instead of having to type the full path to the file.

Here are some of the parameters you just loaded.
For a full list of the firmware's parameters, see [the parameter list](../concepts/parameter-configuration.md).

- **Aircraft configuration**: Sets `FIXED_WING: 0` for multirotor operation
- **RC channels**: configures 8 RC channels with appropriate mappings
- **Mixer configuration**: Uses custom mixer (`PRIMARY_MIXER: 11`) with motor parameters
- **Control gains**: Loads PID gains for roll, pitch, and yaw rate and angle controllers
- **Safety settings**: Sets minimum throttle and failsafe configurations
- **RC override**: Configures attitude and throttle override channels

See the `*.yaml` files in the `rosflight_ros_pkgs/rosflight_sim/params` directory for more information on exactly what parameters were loaded, and what the values were.

## Calibrating the IMU

The IMU (inertial measurement unit) calibration is essential for proper attitude estimation and flight control.
The firmware will not arm until the IMU has been calibrated.

To manually calibrate the IMU, use the calibration service provided by `rosflight_io`:

```bash
# Calibrate IMU
ros2 service call /calibrate_imu std_srvs/srv/Trigger

# Optional: Calibrate barometer
ros2 service call /calibrate_baro std_srvs/srv/Trigger
```

These commands may take a few seconds to complete.
Pay attention to the output from the terminal where you launched the `rosflight_io` node.
You should see something like:
```bash
[rosflight_io-9] [INFO] [1751393533.505232221] [rosflight_io]: Parameter ACC_X_BIAS has new value -0.0507161
[rosflight_io-9] [WARN] [1751393533.505511027] [rosflight_io]: There are unsaved changes to onboard parameters
[rosflight_io-9] [INFO] [1751393533.505577958] [rosflight_io]: Parameter ACC_Y_BIAS has new value 0.215514
[rosflight_io-9] [INFO] [1751393533.505683879] [rosflight_io]: Parameter ACC_Z_BIAS has new value 0.0193333
[rosflight_io-9] [INFO] [1751393533.505775730] [rosflight_io]: [Autopilot]: IMU offsets captured
[rosflight_io-9] [INFO] [1751393533.505820657] [rosflight_io]: Autopilot RECOVERED ERROR: Uncalibrated IMU
[rosflight_io-9] [INFO] [1751393533.506043698] [rosflight_io]: Parameter GYRO_X_BIAS has new value -0.18134
[rosflight_io-9] [INFO] [1751393533.506251903] [rosflight_io]: Parameter GYRO_Y_BIAS has new value -0.142449
[rosflight_io-9] [INFO] [1751393533.506363563] [rosflight_io]: Parameter GYRO_Z_BIAS has new value 0.218309
```

!!! warning "Calibration when armed"
    Calibration will fail if the aircraft is armed.
    ROSflight will also not allow the aircraft to be armed if the calibration is not performed, so this is uncommon.

!!! note "Calibrating for each flight"
    In hardware, IMU calibration should be performed before each flight session.
    This ensures your IMU is well calibrated.
    In sim, if you don't save the parameters to a file (as described below), then you will need to recalibrate.

    Additionally, the aircraft will not arm until firmware parameters are properly loaded.
    These steps are mandatory for every simulation session (if you don't write the params as described below).

## Saving parameters to memory
The ROSflight sim tries to mimic hardware as much as possible.
By default, parameters in the firmware are not saved to flash memory, so they do not persist across reboots of the sim.

To save firmware parameters (including calibration values) to memory, use the `rosflight_io` service call:
```bash
ros2 service call /param_write std_srvs/srv/Trigger
```

In hardware, this service call will write all the firmware's parameters to the SD card, where they will load on boot.
In sim, however, it will generate a directory called `rosflight_memory` in the directory where the command to launch the sim occurred.

To load those same parameters by default the next time that `rosflight_sim` is launched, just launch `rosflight_sim` from the directory that contains the `rosflight_memory` directory.

!!! example
    Suppose I am in a directory with the following file structure:
    ```bash
    rosflight_ws
    ├── build
    ├── install
    ├── log
    └── src
    ```

    And I launch the simulation from this directory using `ros2 launch rosflight_sim multirotor_standalone.launch.py`.
    I then write the params with `ros2 service call /param_write std_srvs/srv/Trigger`.

    My new file structure will be:
    ```bash
    rosflight_ws
    ├── build
    ├── install
    ├── log
    ├── rosflight_memory
    └── src
    ```

    To load those parameters by default when I launch `rosflight_sim` in the future, I just need to launch it from the directory that contains the `rosflight_sim` directory (in this case, `rosflight_ws`).

## Convenience script

We have provided a convenience script that does the above actions for you (loading parameters, calibrating the IMU, and writing params to memory).
To use it, open a new terminal and run:

```bash
# For multirotor
ros2 launch rosflight_sim multirotor_init_firmware.launch.py

# For fixed-wing
ros2 launch rosflight_sim fixedwing_init_firmware.launch.py
```

This launch file performs three critical tasks:

1. **Loads parameters** that are aircraft-specific from YAML files
2. Automatically **calibrates** the IMU
3. **Saves** the loaded parameters to firmware memory

You can use this instead of doing the three actions manually.

## Flying

Once you have followed the steps above, you should be able to fly the aircraft manually.
You can either do this using a keyboard or a supported RC transmitter, connected via USB.

Flying requires you to **arm** the vehicle, meaning that the motors will be active.
Prior to arming, motors will not spin and will not be powered.

If you can arm and take off, you are good to go!
Note that the firmware control loops can be tuned if necessary.
See [the tuning guide](./tuning-performance-in-sim.md) for more information.

### Keyboard Control with VimFly

VimFly provides keyboard-based control for manual flight operations. Launch your simulation with VimFly enabled:

```bash
# Launch with VimFly keyboard control
ros2 launch rosflight_sim multirotor_standalone.launch.py use_vimfly:=true
```

#### VimFly Control Mapping

| Key | Function | Description |
|-----|----------|-------------|
| `a` | Increase Thrust | Raises throttle in 10% increments |
| `s` | Decrease Thrust | Lowers throttle in 10% increments |
| `h` | Roll Left | Commands left roll input |
| `l` | Roll Right | Commands right roll input |
| `j` | Pitch Backward | Commands backward pitch input |
| `k` | Pitch Forward | Commands forward pitch input |
| `d` | Yaw Left (CCW) | Commands counter-clockwise yaw |
| `f` | Yaw Right (CW) | Commands clockwise yaw |
| `t` | Arm/Disarm Toggle | Arms or disarms the aircraft |
| `r` | RC Override Toggle | Toggles RC override mode |

!!! tip "VimFly Window Focus"
    The VimFly window must be in focus to receive keyboard input. Click on the VimFly terminal window to ensure it's active before attempting to control the aircraft.

### RC Transmitter Control

ROSflight supports several types of transmitters or controllers that you can use to fly around in the sim as the RC safety pilot:

#### Supported Transmitters
- **FrSky Taranis Q-X7** (USB connection)
- **RadioMaster TX16S** (USB connection)
- **RadioMaster Boxer** (USB connection)
- **Xbox Controller**
- **RealFlight InterLink Controller**

If one of the supported transmitters is connected via USB **at launch time**, then the sim will default to using that controller instead of the default, **which is no RC connection**.
See the [Hardware Setup](../concepts/hardware-setup.md#joystick) guide for more information on joysticks.

!!! note "Have a joystick not on the list?"
    Joysticks not on the above list may have incorrect mappings.
    If your joystick does not work, and you write your own mapping, please contribute back your new joystick mapping!

## Troubleshooting

### Common Issues and Solutions

??? warning "Aircraft Won't Arm"

    **Symptoms**: Aircraft refuses to arm despite proper setup

    **Possible Causes and Solutions**:

    - **IMU not calibrated**: Run `ros2 service call /calibrate_imu std_srvs/srv/Trigger`
    - **Parameters not loaded**: Execute the appropriate firmware initialization launch file
    - **RC not connected**: Verify RC transmitter connection or use VimFly
    - **Safety checks failing**: Check for error messages in the `rosflight_io` node output

??? warning "VimFly Not Responding"

    **Symptoms**: Keyboard inputs not controlling the aircraft

    **Solutions**:

    - **Window focus**: Click on the VimFly terminal window to ensure it has focus
    - **Pygame dependency**: Verify pygame is installed: `pip install pygame`
    - **Launch parameter**: Ensure `use_vimfly:=true` is set in launch command
    - **RC override**: Check if RC override is enabled and disable if necessary

??? warning "RC Transmitter Not Detected"

    **Symptoms**: No RC input detected in simulation

    **Solutions**:

    - **USB connection**: Verify transmitter is connected via USB
    - **Device permissions**: Check USB device permissions and udev rules
    - **Joystick mode**: Ensure transmitter is in USB joystick mode
    - **Monitor output**: Use `ros2 topic echo /rc_raw` to verify RC signals

??? warning "Poor Flight Performance"

    **Symptoms**: Aircraft is unstable or difficult to control

    **Solutions**:

    - **Parameter verification**: Ensure correct firmware parameters are loaded
    - **IMU recalibration**: Recalibrate IMU if attitude estimation appears incorrect
    - **Control gains**: Check if PID gains are appropriate for aircraft type. See [the tuning guide](./tuning-performance-in-sim.md)
    - **Simulation rate**: Verify simulation is running at proper real-time rate

??? warning "Simulation Lag or Stuttering"

    **Symptoms**: Jerky or delayed response to control inputs

    **Solutions**:

    - **System resources**: Close unnecessary applications to free CPU/memory
    - **Simulation complexity**: Reduce visual complexity in Gazebo simulation
    - **Network latency**: Check for network issues if using remote display
    - **Hardware acceleration**: Ensure graphics drivers are properly installed

## Review

You have successfully completed the manual flight tutorial for ROSflight simulation. You should now be able to:

- **Initialize Firmware**: Load appropriate parameters for multirotor or fixed-wing aircraft
- **Calibrate Sensors**: Perform IMU calibration for proper attitude estimation
- **Control Aircraft**: Fly using either VimFly keyboard controls or RC transmitter
- **Troubleshoot Issues**: Diagnose and resolve common simulation problems

## Next Steps

Once you have the simulator running, you can:

1. **[Autonomous flight](./setting-up-roscopter-in-sim.md)**: Integrate with the ROScopter or ROSplane autonomy stacks
1. **[Custom applications](../../developer-guide/contribution-guidelines.md)**: Use your own ROS2 nodes with ROSflight
1. **[Parameter/Gain tuning](./tuning-performance-in-sim.md)**: Use the RQT plugins to tune PID controllers and other parameters

### Additional Resources

- [ROSflight Parameter Reference](../concepts/parameter-configuration.md): Detailed parameter descriptions
- [Hardware Setup Guide](../concepts/hardware-setup.md): Preparing real hardware for flight
