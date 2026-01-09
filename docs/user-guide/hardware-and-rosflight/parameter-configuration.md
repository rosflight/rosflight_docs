# Parameters

The ROSflight firmware has several dozen parameters which it uses to customize performance. Parameters are considered semi-static variables. That is, parameters do not change during flight, but they may change between vehicles. Examples of parameters you may wish to change are:

* Fixed-wing vehicle flag
* PID gains
* Mixer choice
* IMU low-pass filter constant
* RC receiver type (PPM or SBUS)

and so on. Access to all parameters is enabled via ROS2 services advertised by `rosflight_io` while the flight controller is connected.

## Parameter Interface

### Getting Parameter Values

Sometimes it is handy to ask the flight controller what the current value of a parameter is. This is accomplished using the `param_get` service. As an example, let's retrieve the roll angle controller proportional (P) gain.

```
ros2 service call /param_get rosflight_msgs/srv/ParamGet "{name: "PID_ROLL_ANG_P"}"
```

You should get a response similar to the following (this happens to be the default value with floating-point error):

```
exists: True
value: 0.15000000596
```

### Changing Parameters

Parameters are changed via the `param_set` service. As an example, let's change the roll angle controller P gain. (I will assume that the flight controller is connected and `rosflight_io` is running in the root namespace.)

```
ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: "PID_ROLL_ANG_P", value: 0.08}"
```

You should get a prompt from `rosflight_io` saying
```
[ INFO] [1491672408.585339558]: Parameter PID_ROLL_ANG_P has new value 0.08
[ WARN] [1491672408.585508849]: There are unsaved changes to onboard parameters
```

Notice that the parameters have been set, but not saved. Parameter changes take effect immediately, however they will not persist over a reboot unless you *write* them to the non-volatile memory. This brings us to the next task.

#### Changing Parameters via `rosflight_io` ROS2 params

The `rosflight_io` node has some firmware parameters exposed via the ROS2 parameter interface, enabling quick configuration of *some* of the firmware's parameters.
This means that changing these `rosflight_io` parameters via the standard ROS2 parameter configuration will automatically change them in the firmware.

Currently, only the controller gains have been exposed to `rosflight_io`'s parameters.
To expose more, see the `rosflight_io.cpp` file.

### Writing Parameters

To ensure that parameter values persist between reboots, you must write the parameters to the non-volatile memory. This is done by calling `param_write`

```
ros2 service call /param_write std_srvs/srv/Trigger
```

`rosflight_io` should then respond with
```
[ INFO] [1491672597.123201952]: Param write succeeded
[ INFO] [1491672597.123452908]: Onboard parameters have been saved
```

!!! warning
    It is highly recommended that you write parameters before arming and flying the vehicle. Among other things, this will ensure that in the rare case that a hard fault is encountered and the flight controller must reboot during flight, the correct configuration will be loaded on reboot.

!!! tip
    Parameter writing can only happen if the flight controller is disarmed. If the param write failed for some reason, you may want to make sure your FC is disarmed and try again.

### Backing Up and Loading Parameters from File

It is good practice to back up your parameter configuration in case you have to re-flash your firmware or you want to share configurations between vehicles. We can do this via the `param_save_to_file` and `param_load_from_file` services.

First, let's back up our current parameter configuration:

```
ros2 service call /param_save_to_file rosflight_msgs/srv/ParamFile "{filename: "~/parameters.yaml"}"
```

Parameters are saved in YAML format. You must also specify the absolute file name of where you would like your parameters to be saved. The current active set of parameters will be saved, regardless of what is saved in non-volatile memory on the flight controller.

Now, let's say we want to re-load this parameter file
```
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename: "~/parameters.yml"}"
```
Again, you must specify the absolute file name of the file to be loaded.


## Fixed-Wing Parameter Configuration

Because ROSflight ships with default parameters for multirotors, you will probably want to change the following parameters if you want to fly a fixed-wing aircraft.


| Parameter | Description | Type | Fixed-Wing Value
|-----------|-------------|------|---------------|
| MOTOR_PWM_UPDATE | Refresh rate of motor commands to motors and servos (Hz) - See motor documentation | int |  50 |
| ARM_SPIN_MOTORS | Enforce MOTOR_IDLE_PWM | int |  false |
| MOTOR_IDLE_THR | min throttle command sent to motors when armed (Set above 0.1 to spin when armed) | float |  0.1 |
| ARM_CHANNEL | RC switch channel mapped to arming [0 indexed, -1 to disable] | int |  4 |
| FIXED_WING | switches on passthrough commands for fixed-wing operation | int |  true |
| PRIMARY_MIXER | Which primary mixer to choose - See [Mixer documentation](../rosflight-firmware/mixer.md) | int | 9 or 10  |
| ELEVATOR_REV | reverses elevator servo output | int |  0 or 1 |
| AIL_REV | reverses aileron servo output | int |  0 or 1 |
| RUDDER_REV | reverses rudder servo output | int |  0 or 1 |
| CAL_GYRO_ARM | Calibrate gyros when arming - generally only for multirotors | int |  false |


## Description of all Parameters

This is a list of all ROSflight parameters, including their types, default values, and minimum and maximum recommended values:

| Parameter | Description | Type | Default Value | Min | Max |
|-----------|-------------|------|---------------|-----|-----|
| BAUD_RATE | Baud rate of MAVlink communication with companion computer | int |  921600 | 9600 | 921600 |
| SERIAL_DEVICE | Serial Port (for supported devices) | int |  0 | 0 | 3 |
| AIR_DENSITY | Density of the air (kg/m^3) | float |  1.225f | 0 | 1000.0 |
| NUM_MOTORS | Number of vertical-facing motors on the vehicle | int |  4 | 1 | 8 |
| MOTOR_RESISTANCE | Electrical resistance of the motor windings (ohms) | float |  0.042f | 0 | 1000.0 |
| MOTOR_KV | Back emf constant of the motor in SI units (V/rad/s) | float |  0.01706f | 0 | 1000.0 |
| NO_LOAD_CURRENT | No-load current of the motor in amps | float |  1.5 | 0 | 1000.0 |
| PROP_DIAMETER | Diameter of the propeller in meters | float |  0.381f | 0 | 1.0 |
| PROP_CT | Thrust coefficient of the propeller | float |  0.075f | 0 | 100.0 |
| PROP_CQ | Torque coefficient of the propeller | float |  0.0045f | 0 | 100.0 |
| VOLT_MAX | Maximum voltage of the battery (V) | float |  25.0f | 0 | 100.0 |
| USE_MOTOR_PARAM | Flag to use motor parameters in the mixer | int |  false | 0 | 1 |
| PRI_MIXER_OUT_0 | Output type of mixer output 0. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_1 | Output type of mixer output 1. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_2 | Output type of mixer output 2. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_3 | Output type of mixer output 3. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_4 | Output type of mixer output 4. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_5 | Output type of mixer output 5. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_6 | Output type of mixer output 6. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_7 | Output type of mixer output 7. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_8 | Output type of mixer output 8. | int |  0 | 0 | 1 |
| PRI_MIXER_OUT_9 | Output type of mixer output 9. | int |  0 | 0 | 1 |
| PRI_MIXER_PWM_0 | PWM frequenct output for mixer output 0 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_1 | PWM frequenct output for mixer output 1 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_2 | PWM frequenct output for mixer output 2 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_3 | PWM frequenct output for mixer output 3 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_4 | PWM frequenct output for mixer output 4 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_5 | PWM frequenct output for mixer output 5 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_6 | PWM frequenct output for mixer output 6 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_7 | PWM frequenct output for mixer output 7 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_8 | PWM frequenct output for mixer output 8 | float |  0 | 0 | 490 |
| PRI_MIXER_PWM_9 | PWM frequenct output for mixer output 9 | float |  0 | 0 | 490 |
| PRI_MIXER_0_0 | Value of the custom mixer at element [0,0] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_0 | Value of the custom mixer at element [1,0] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_0 | Value of the custom mixer at element [2,0] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_0 | Value of the custom mixer at element [3,0] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_0 | Value of the custom mixer at element [4,0] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_0 | Value of the custom mixer at element [5,0] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_1 | Value of the custom mixer at element [0,1] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_1 | Value of the custom mixer at element [1,1] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_1 | Value of the custom mixer at element [2,1] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_1 | Value of the custom mixer at element [3,1] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_1 | Value of the custom mixer at element [4,1] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_1 | Value of the custom mixer at element [5,1] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_2 | Value of the custom mixer at element [0,2] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_2 | Value of the custom mixer at element [1,2] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_2 | Value of the custom mixer at element [2,2] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_2 | Value of the custom mixer at element [3,2] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_2 | Value of the custom mixer at element [4,2] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_2 | Value of the custom mixer at element [5,2] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_3 | Value of the custom mixer at element [0,3] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_3 | Value of the custom mixer at element [1,3] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_3 | Value of the custom mixer at element [2,3] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_3 | Value of the custom mixer at element [3,3] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_3 | Value of the custom mixer at element [4,3] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_3 | Value of the custom mixer at element [5,3] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_4 | Value of the custom mixer at element [0,4] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_4 | Value of the custom mixer at element [1,4] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_4 | Value of the custom mixer at element [2,4] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_4 | Value of the custom mixer at element [3,4] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_4 | Value of the custom mixer at element [4,4] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_4 | Value of the custom mixer at element [5,4] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_5 | Value of the custom mixer at element [0,5] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_5 | Value of the custom mixer at element [1,5] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_5 | Value of the custom mixer at element [2,5] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_5 | Value of the custom mixer at element [3,5] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_5 | Value of the custom mixer at element [4,5] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_5 | Value of the custom mixer at element [5,5] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_6 | Value of the custom mixer at element [0,6] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_6 | Value of the custom mixer at element [1,6] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_6 | Value of the custom mixer at element [2,6] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_6 | Value of the custom mixer at element [3,6] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_6 | Value of the custom mixer at element [4,6] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_6 | Value of the custom mixer at element [5,6] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_7 | Value of the custom mixer at element [0,7] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_7 | Value of the custom mixer at element [1,7] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_7 | Value of the custom mixer at element [2,7] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_7 | Value of the custom mixer at element [3,7] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_7 | Value of the custom mixer at element [4,7] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_7 | Value of the custom mixer at element [5,7] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_8 | Value of the custom mixer at element [0,8] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_8 | Value of the custom mixer at element [1,8] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_8 | Value of the custom mixer at element [2,8] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_8 | Value of the custom mixer at element [3,8] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_8 | Value of the custom mixer at element [4,8] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_8 | Value of the custom mixer at element [5,8] | float |  0.0f | -inf | inf |
| PRI_MIXER_0_9 | Value of the custom mixer at element [0,9] | float |  0.0f | -inf | inf |
| PRI_MIXER_1_9 | Value of the custom mixer at element [1,9] | float |  0.0f | -inf | inf |
| PRI_MIXER_2_9 | Value of the custom mixer at element [2,9] | float |  0.0f | -inf | inf |
| PRI_MIXER_3_9 | Value of the custom mixer at element [3,9] | float |  0.0f | -inf | inf |
| PRI_MIXER_4_9 | Value of the custom mixer at element [4,9] | float |  0.0f | -inf | inf |
| PRI_MIXER_5_9 | Value of the custom mixer at element [5,9] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_0 | Value of the custom mixer at element [0,0] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_0 | Value of the custom mixer at element [1,0] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_0 | Value of the custom mixer at element [2,0] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_0 | Value of the custom mixer at element [3,0] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_0 | Value of the custom mixer at element [4,0] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_0 | Value of the custom mixer at element [5,0] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_1 | Value of the custom mixer at element [0,1] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_1 | Value of the custom mixer at element [1,1] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_1 | Value of the custom mixer at element [2,1] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_1 | Value of the custom mixer at element [3,1] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_1 | Value of the custom mixer at element [4,1] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_1 | Value of the custom mixer at element [5,1] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_2 | Value of the custom mixer at element [0,2] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_2 | Value of the custom mixer at element [1,2] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_2 | Value of the custom mixer at element [2,2] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_2 | Value of the custom mixer at element [3,2] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_2 | Value of the custom mixer at element [4,2] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_2 | Value of the custom mixer at element [5,2] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_3 | Value of the custom mixer at element [0,3] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_3 | Value of the custom mixer at element [1,3] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_3 | Value of the custom mixer at element [2,3] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_3 | Value of the custom mixer at element [3,3] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_3 | Value of the custom mixer at element [4,3] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_3 | Value of the custom mixer at element [5,3] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_4 | Value of the custom mixer at element [0,4] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_4 | Value of the custom mixer at element [1,4] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_4 | Value of the custom mixer at element [2,4] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_4 | Value of the custom mixer at element [3,4] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_4 | Value of the custom mixer at element [4,4] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_4 | Value of the custom mixer at element [5,4] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_5 | Value of the custom mixer at element [0,5] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_5 | Value of the custom mixer at element [1,5] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_5 | Value of the custom mixer at element [2,5] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_5 | Value of the custom mixer at element [3,5] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_5 | Value of the custom mixer at element [4,5] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_5 | Value of the custom mixer at element [5,5] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_6 | Value of the custom mixer at element [0,6] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_6 | Value of the custom mixer at element [1,6] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_6 | Value of the custom mixer at element [2,6] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_6 | Value of the custom mixer at element [3,6] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_6 | Value of the custom mixer at element [4,6] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_6 | Value of the custom mixer at element [5,6] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_7 | Value of the custom mixer at element [0,7] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_7 | Value of the custom mixer at element [1,7] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_7 | Value of the custom mixer at element [2,7] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_7 | Value of the custom mixer at element [3,7] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_7 | Value of the custom mixer at element [4,7] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_7 | Value of the custom mixer at element [5,7] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_8 | Value of the custom mixer at element [0,8] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_8 | Value of the custom mixer at element [1,8] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_8 | Value of the custom mixer at element [2,8] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_8 | Value of the custom mixer at element [3,8] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_8 | Value of the custom mixer at element [4,8] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_8 | Value of the custom mixer at element [5,8] | float |  0.0f | -inf | inf |
| SEC_MIXER_0_9 | Value of the custom mixer at element [0,9] | float |  0.0f | -inf | inf |
| SEC_MIXER_1_9 | Value of the custom mixer at element [1,9] | float |  0.0f | -inf | inf |
| SEC_MIXER_2_9 | Value of the custom mixer at element [2,9] | float |  0.0f | -inf | inf |
| SEC_MIXER_3_9 | Value of the custom mixer at element [3,9] | float |  0.0f | -inf | inf |
| SEC_MIXER_4_9 | Value of the custom mixer at element [4,9] | float |  0.0f | -inf | inf |
| SEC_MIXER_5_9 | Value of the custom mixer at element [5,9] | float |  0.0f | -inf | inf |
| SYS_ID | Mavlink System ID | int |  1 | 1 | 255 |
| PID_ROLL_RATE_P | Roll Rate Proportional Gain | float |  0.070f | 0.0 | 1000.0 |
| PID_ROLL_RATE_I | Roll Rate Integral Gain | float |  0.000f | 0.0 | 1000.0 |
| PID_ROLL_RATE_D | Roll Rate Derivative Gain | float |  0.000f | 0.0 | 1000.0 |
| PID_PITCH_RATE_P | Pitch Rate Proportional Gain | float |  0.070f | 0.0 | 1000.0 |
| PID_PITCH_RATE_I | Pitch Rate Integral Gain | float |  0.0000f | 0.0 | 1000.0 |
| PID_PITCH_RATE_D | Pitch Rate Derivative Gain | float |  0.0000f | 0.0 | 1000.0 |
| PID_YAW_RATE_P | Yaw Rate Proportional Gain | float |  0.25f | 0.0 | 1000.0 |
| PID_YAW_RATE_I | Yaw Rate Integral Gain | float |  0.0f | 0.0 | 1000.0 |
| PID_YAW_RATE_D | Yaw Rate Derivative Gain | float |  0.0f | 0.0 | 1000.0 |
| PID_ROLL_ANG_P | Roll Angle Proportional Gain | float |  0.15f | 0.0 | 1000.0 |
| PID_ROLL_ANG_I | Roll Angle Integral Gain | float |  0.0f | 0.0 | 1000.0 |
| PID_ROLL_ANG_D | Roll Angle Derivative Gain | float |  0.05f | 0.0 | 1000.0 |
| PID_PITCH_ANG_P | Pitch Angle Proportional Gain | float |  0.15f | 0.0 | 1000.0 |
| PID_PITCH_ANG_I | Pitch Angle Integral Gain | float |  0.0f | 0.0 | 1000.0 |
| PID_PITCH_ANG_D | Pitch Angle Derivative Gain | float |  0.05f | 0.0 | 1000.0 |
| X_EQ_TORQUE | Equilibrium torque added to output of controller on x axis | float |  0.0f | -1.0 | 1.0 |
| Y_EQ_TORQUE | Equilibrium torque added to output of controller on y axis | float |  0.0f | -1.0 | 1.0 |
| Z_EQ_TORQUE | Equilibrium torque added to output of controller on z axis | float |  0.0f | -1.0 | 1.0 |
| PID_TAU | Dirty Derivative time constant - See controller documentation | float |  0.05f | 0.0 | 1.0 |
| MOTOR_PWM_UPDATE | Overrides default PWM rate specified by mixer if non-zero - Requires reboot to take effect | int |  0 | 0 | 490 |
| MOTOR_IDLE_THR | min throttle command sent to motors when armed (Set above 0.1 to spin when armed) | float |  0.1 | 0.0 | 1.0 |
| FAILSAFE_THR | Throttle sent to motors in failsafe condition (set just below hover throttle) | float |  -1.0 | 0.0 | 1.0 |
| ARM_SPIN_MOTORS | Enforce MOTOR_IDLE_THR | int |  true | 0 | 1 |
| FILTER_INIT_T | Time in ms to initialize estimator | int |  3000 | 0 | 100000 |
| FILTER_KP_ACC | estimator proportional gain on accel-based error - See estimator documentation | float |  0.5f | 0 | 10.0 |
| FILTER_KI | estimator integral gain - See estimator documentation | float |  0.01f | 0 | 1.0 |
| FILTER_KP_EXT | estimator proportional gain on external attitude-based error - See estimator documentation | float |  1.5f | 0 | 10.0 |
| FILTER_ACCMARGIN | allowable accel norm margin around 1g to determine if accel is usable | float |  0.1f | 0 | 1.0 |
| FILTER_QUAD_INT | Perform a quadratic averaging of LPF gyro data prior to integration (adds ~20 us to estimation loop on F1 processors) | int |  1 | 0 | 1 |
| FILTER_MAT_EXP | 1 - Use matrix exponential to improve gyro integration (adds ~90 us to estimation loop in F1 processors) 0 - use euler integration | int |  1 | 0 | 1 |
| FILTER_USE_ACC | Use accelerometer to correct gyro integration drift (adds ~70 us to estimation loop) | int |  1 | 0 | 1 |
| CAL_GYRO_ARM | True if desired to calibrate gyros on arm | int |  false | 0 | 1 |
| GYROXY_LPF_ALPHA | Low-pass filter constant on gyro X and Y axes - See estimator documentation | float |  0.3f | 0 | 1.0 |
| GYROZ_LPF_ALPHA | Low-pass filter constant on gyro Z axis - See estimator documentation | float |  0.3f | 0 | 1.0 |
| ACC_LPF_ALPHA | Low-pass filter constant on all accel axes - See estimator documentation | float |  0.5f | 0 | 1.0 |
| GYRO_X_BIAS | Constant x-bias of gyroscope readings | float |  0.0f | -1.0 | 1.0 |
| GYRO_Y_BIAS | Constant y-bias of gyroscope readings | float |  0.0f | -1.0 | 1.0 |
| GYRO_Z_BIAS | Constant z-bias of gyroscope readings | float |  0.0f | -1.0 | 1.0 |
| ACC_X_BIAS | Constant x-bias of accelerometer readings | float |  0.0f | -2.0 | 2.0 |
| ACC_Y_BIAS | Constant y-bias of accelerometer readings | float |  0.0f | -2.0 | 2.0 |
| ACC_Z_BIAS | Constant z-bias of accelerometer readings | float |  0.0f | -2.0 | 2.0 |
| ACC_X_TEMP_COMP | Linear x-axis temperature compensation constant | float |  0.0f | -2.0 | 2.0 |
| ACC_Y_TEMP_COMP | Linear y-axis temperature compensation constant | float |  0.0f | -2.0 | 2.0 |
| ACC_Z_TEMP_COMP | Linear z-axis temperature compensation constant | float |  0.0f | -2.0 | 2.0 |
| MAG_A11_COMP | Soft iron compensation constant | float |  1.0f | -999.0 | 999.0 |
| MAG_A12_COMP | Soft iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_A13_COMP | Soft iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_A21_COMP | Soft iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_A22_COMP | Soft iron compensation constant | float |  1.0f | -999.0 | 999.0 |
| MAG_A23_COMP | Soft iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_A31_COMP | Soft iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_A32_COMP | Soft iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_A33_COMP | Soft iron compensation constant | float |  1.0f | -999.0 | 999.0 |
| MAG_X_BIAS | Hard iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_Y_BIAS | Hard iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| MAG_Z_BIAS | Hard iron compensation constant | float |  0.0f | -999.0 | 999.0 |
| BARO_BIAS | Barometer measurement bias (Pa) | float |  0.0f | 0 | inf |
| GROUND_LEVEL | Altitude of ground level (m) | float |  1387.0f | -1000 | 10000 |
| DIFF_PRESS_BIAS | Differential Pressure Bias (Pa) | float |  0.0f | -10 | 10 |
| RC_TYPE | Type of RC input 0 - PPM, 1 - SBUS | int |  0 | 0 | 1 |
| RC_X_CHN | RC input channel mapped to x-axis commands [0 - indexed] | int |  0 | 0 | 3 |
| RC_Y_CHN | RC input channel mapped to y-axis commands [0 - indexed] | int |  1 | 0 | 3 |
| RC_Z_CHN | RC input channel mapped to z-axis commands [0 - indexed] | int |  3 | 0 | 3 |
| RC_F_CHN | RC input channel mapped to F-axis commands [0 - indexed] | int |  2 | 0 | 3 |
| RC_F_AXIS | NED axis that RC F-channel gets mapped to 0 - X, 1 - Y, 2 - Z | int |  2 | 0 | 2 |
| RC_ATT_OVRD_CHN | RC switch mapped to attitude override [0 indexed, -1 to disable] | int |  4 | 4 | 7 |
| RC_THR_OVRD_CHN | RC switch channel mapped to throttle override [0 indexed, -1 to disable] | int |  4 | 4 | 7 |
| RC_ATT_CTRL_CHN | RC switch channel mapped to attitude control type [0 indexed, -1 to disable] | int | -1 | 4 | 7 |
| ARM_CHANNEL | RC switch channel mapped to arming (only if PARAM_ARM_STICKS is false) [0 indexed, -1 to disable] | int | -1 | 4 | 7 |
| RC_NUM_CHN | number of RC input channels | int |  6 | 1 | 8 |
| SWITCH_5_DIR | RC switch 5 toggle direction | int |  1 | -1 | 1 |
| SWITCH_6_DIR | RC switch 6 toggle direction | int |  1 | -1 | 1 |
| SWITCH_7_DIR | RC switch 7 toggle direction | int |  1 | -1 | 1 |
| SWITCH_8_DIR | RC switch 8 toggle direction | int |  1 | -1 | 1 |
| RC_OVRD_DEV | RC stick deviation from center for override | float |  0.1 | 0.0 | 1.0 |
| OVRD_LAG_TIME | RC stick deviation lag time before returning control (ms) | int |  1000 | 0 | 100000 |
| MIN_THROTTLE | Take minimum throttle between RC and computer at all times | int |  true | 0 | 1 |
| RC_MAX_THR | Maximum throttle command sent by full deflection of RC sticks, to maintain controllability during aggressive maneuvers | float |  0.7f | 0.0 | 1.0 |
| RC_ATT_MODE | Attitude mode for RC sticks (0: rate, 1: angle). Overridden if RC_ATT_CTRL_CHN is set. | int |  1 | 0 | 1 |
| RC_MAX_ROLL | Maximum roll angle command sent by full deflection of RC sticks | float |  0.786f | 0.0 | 3.14159 |
| RC_MAX_PITCH | Maximum pitch angle command sent by full stick deflection of RC sticks | float |  0.786f | 0.0 | 3.14159 |
| RC_MAX_ROLLRATE | Maximum roll rate command sent by full stick deflection of RC sticks | float |  3.14159f | 0.0 | 9.42477796077 |
| RC_MAX_PITCHRATE | Maximum pitch command sent by full stick deflection of RC sticks | float |  3.14159f | 0.0 | 3.14159 |
| RC_MAX_YAWRATE | Maximum pitch command sent by full stick deflection of RC sticks | float |  1.507f | 0.0 | 3.14159 |
| PRIMARY_MIXER | Which mixer to choose for primary mixer - See Mixer documentation | int |  Mixer::INVALID_MIXER | 0 | 11 |
| SECONDARY_MIXER | Which mixer to choose for secondary mixer - See Mixer documentation | int |  Mixer::INVALID_MIXER | 0 | 11 |
| FIXED_WING | switches on pass-through commands for fixed-wing operation | int |  false | 0 | 1 |
| ELEVATOR_REV | reverses elevator servo output | int |  0 | 0 | 1 |
| AIL_REV | reverses aileron servo output | int |  0 | 0 | 1 |
| RUDDER_REV | reverses rudder servo output | int |  0 | 0 | 1 |
| FC_ROLL | roll angle (deg) of flight controller wrt aircraft body | float |  0.0f | 0 | 360 |
| FC_PITCH | pitch angle (deg) of flight controller wrt aircraft body | float |  0.0f | 0 | 360 |
| FC_YAW | yaw angle (deg) of flight controller wrt aircraft body | float |  0.0f | 0 | 360 |
| ARM_THRESHOLD | RC deviation from max/min in yaw and throttle for arming and disarming check (us) | float |  0.15 | 0 | 500 |
| BATT_VOLT_MULT | Multiplier for the voltage sensor | float |  0.0f | 0 | inf |
| BATT_CURR_MULT | Multiplier for the current sensor | float |  0.0f | 0 | inf |
| BATT_VOLT_ALPHA | Alpha value for the low pass filter on the reported battery voltage | float |  0.995f | 0 | 1 |
| BATT_CURR_ALPHA | Alpha value for the low pass filter on the reported battery current | float |  0.995f | 0 | 1 |
| OFFBOARD_TIMEOUT | Timeout in milliseconds for offboard commands, after which RC override is activated | int |  100 | 0 | 100000 |
