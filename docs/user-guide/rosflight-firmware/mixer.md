# Mixer

A controller is responsible for computing commands that drive a system to a given reference.
However, in general the output of a controller is not raw motor commands.
For example, given a quadrotor vehicle, the [ROSflight firmware rate controller](TODO:insertlinkhere) takes in desired angular rates and outputs desired forces and torques that will achieve those rates.
These desired forces and torques are not individual actuator commands.
Thus, these desired forces and torques must be correctly *mapped* to each individual actuator.

The *mixer* is responsible for mapping, or mixing, the output of a controller to the individual actuator commands of a MAV.
The mixer is highly frame-specific: a quadrotor mixer will be different than a hexarotor mixer since the hexarotor has 2 additional rotors and all six rotors are in physically different locations relative to the vehicle center of mass.

!!! danger
    If the mixer is not set correctly, it will lead to a crash.
    Make sure it is set properly for your airframe!

In ROSflight, the mixer is a matrix and operates on the inputs $u \in \mathbb{R}^m$ as
$$
\tau = M^\dagger u
$$
where $\tau \in \mathbb{R}^n$ is the vector of individual motor commands, $M \in \mathbb{R}^{m\times n}$ is the mixing matrix, and $(\cdot)^\dagger$ is the Moore-Penrose pseudoinverse.

Note that $n$ denotes the number of possible output commands and $m$ denotes the size of the input commands, which for ROSflight is 6.
Note that $m=6$ was arbitrarily chosen, but conveniently corresponds to the number of forces and torques for a 6-DoF body.
The number of possible output commands is dependent on the number of PWM hardware outputs.

!!! note

    The rest of this guide focuses on **using** the ROSflight mixer.
    Please see the [ROSflight 2.0 paper](https://arxiv.org/abs/2510.00995) for more information on the derivation and details of the ROSflight mixer.

    Additionally, this guide tries not to repeat information found in the paper, so we recommend reading through that paper to get an overview of the ROSflight mixer.

## ROSflight mixer implementation details
Currently supported hardware for the ROSflight firmware has a maximum of 10 output channels.
Thus, the ROSflight mixing matrix is a 6x10 matrix.

In addition to storing these 60 values, each ROSflight mixer has a header with the following information:

- PWM rate (in Hz) for each output channel
- Output type (motor, servo, GPIO, or auxiliary)

Users usually won't have to adjust these header values, unless they [define a custom mixer](#defining-a-custom-mixer) as described below.

## Selecting a primary mixer

A primary mixer must be selected before the firmware will allow arming of the vehicle.
The primary mixer is used by the RC safety pilot and thus must be a mixer that allows the RC safety pilot to control the vehicle.
Specifically, the `PRIMARY_MIXER` parameter must be set to a valid value.

As described in the [ROSflight 2.0 paper](https://arxiv.org/abs/2510.00995), ROSflight offers some pre-computed, "canned" mixers that can be used off the shelf for a variety of common multirotor and fixedwing airframes.
These mixers do not take into account all the parameters of your system (i.e. motor and propeller parameters), but instead are based solely on the geometry of the airframe.
If you want a more accurate mixer, or have easy access to the motor and prop parameters of your system, then we recommend using a [custom mixer](#defining-a-custom-mixer).

The desired mixer must be chosen by [setting the `PRIMARY_MIXER` parameter](../hardware-and-rosflight/parameter-configuration.md) to one of the following valid values.


| # | Mixer |
|---|---------|
| 0 | ESC calibration |
| 1 | Quad + |
| 2 | Quad X |
| 3 | Hex + |
| 4 | Hex X |
| 5 | Octo + |
| 6 | Octo X |
| 7 | Y6 |
| 8 | X8 |
| 9 | Fixed-wing (traditional AETR) |
| 10 | Inverted V-tail fixedwing (like the RMRC Anaconda frame) |
| 11 | Custom mixer |

The associated motor layouts are shown below for each mixer.
The **ESC calibration** mixer directly outputs the throttle command equally to each motor, and can be used for calibrating the ESCs.

![Mixer_1](../images/mixers_1.png)

## Selecting a secondary mixer
As described in the [ROSflight 2.0 paper](https://arxiv.org/abs/2510.00995), the secondary mixer is used by offboard control setpoints (sent by the companion computer).
The secondary mixer can be selected by setting the `SECONDARY_MIXER` parameter to one of the valid values in the previous table.

The secondary mixer allows flexibility for more advanced mixing schemes while still having a functional mixer available to a safety pilot.
For example, a secondary mixer on a quadrotor could be configured as an identity matrix.
This would allow motor commands to be sent directly from the offboard computer to the motors.
In this case, the primary mixer could be set to mixer 2 so that the RC safety pilot can use the ROSflight angle/rate controllers, whose output is properly mixed to actuator commands by the mixer 2.

!!! note
    If the secondary mixer is not specified (or set to an invalid value), then it will default to the primary mixer.
    If you don't need a distinction between the mixer used by the RC safety pilot and the onboard computer, then you do not need to set the secondary mixer parameter.

## Mixing the primary and secondary mixers
In ROSflight, the RC safety pilot has the ability to independently override the attitude commands, the throttle commands, or both.
See the [RC configuration page](../hardware-and-rosflight/rc-configuration.md) for more information.

Remember that the RC commands use the primary mixer and offboard control commands use the secondary mixer.
Thus, when only RC attitude override is active (or conversely only RC throttle override) the mixer actually used by ROSflight to mix controller commands is a combination of the primary and secondary mixers.

When this happens, the primary and secondary mixers are "mixed" as shown in the following image.

<figure markdown="span">
    ![RC Mixer Configuration](../images/primary_secondary_mixer_mixing.svg){ width="1200" loading=lazy }
    <figcaption>Diagram of how the actual mixer used by ROSflight, $M$, is "mixed" with the primary and secondary mixers, depending on the status of RC override.</figcaption>
</figure>

The `mixer_to_use_` structure represents the mixer that is actually used when computing the output.
When just RC throttle override is active (i.e. RC pilot controls the throttle while companion computer controls attitude), the $F$ rows of the `mixer_to_use_` are mapped to the $F$ rows of the primary mixer.
When just RC attitude override is active (i.e. RC pilot controls attitude while companion computer controls throttle), the $Q$ rows of the `mixer_to_use_` are mapped to the $Q$ rows of the primary mixer.

The header, which includes the default PWM rate and the output type for each output channel, is always set to the header of the primary mixer.


## Using Motor Parameters
This section gives some background for the `USE_MOTOR_PARAM` firmware parameter and when it should be set.

As described in _Small Unmanned Aircraft: Theory and Practice_ by Beard and McLain, the mixing matrix is formed using equations from propeller theory, resulting in a set of equations that set the desired forces and torques equal to the square of the angular speeds of the propellers.

This results in equations of the form
$$
M^\dagger u = \tau \propto 
\begin{bmatrix}
\Omega_1^2 & \Omega_2^2 & \dots & \Omega_{10}^2
\end{bmatrix}^T
$$
where $\Omega_i^2$ is the squared angular speed of motor $i$.
Note that constant factors were omitted in the above equation for simplicity, resulting in the proportionality.

If the motor and propeller parameters are known, then the desired voltage setting can be computed from these squared angular speeds.
The PWM motor output for each motor can be computed from these desired voltage settings.

If the `USE_MOTOR_PARAM=1`, then the firmware will assume that the mixed output ($\tau$) is a vector of squared angular speeds ($\Omega^2$), and will use the stored motor and propeller parameters to compute motor commands ($\delta$).

If the motor and propeller parameters are not known, then some simplifying assumptions are made to compute the desired throttle settings for each motor from the desired forces and torques.
In other words, these assumptions result in equations that look like
$$
M^\dagger u = \tau =
\begin{bmatrix}
\delta_1 & \delta_2 & \dots & \delta_{10}
\end{bmatrix}^T
$$
where $\delta_i$ is the $i$th PWM motor output.
**This is the form for all precomputed ("canned") mixers.**

Therefore, if using a canned mixer, set `USE_MOTOR_PARAM=0` so that the firmware interprets the output of the mixer as $\delta$ commands, not $\Omega^2$ commands.

See [the ROSflight 2.0 publication](https://arxiv.org/abs/2510.00995) for more information on the specific assumptions and more information on the above topics.


The canned mixing matrices assume that `USE_MOTOR_PARAM=0`.
Using a canned mixer matrix and setting `USE_MOTOR_PARAM=1` (i.e. specifying that you want to mix with motor and propeller parameters) will cause the outputs to be scaled incorrectly.
It is not required to use motor and propeller parameters when using a custom mixing matrix, but make sure your custom mixer makes sense.

Also, if you selected a custom mixer and used the motor parameters to generate the mixer, make sure you set `USE_MOTOR_PARAM=1`. Otherwise, the outputs will likely be scaled incorrectly.

!!! abstract "Summary"

    If using a canned mixer, set `USE_MOTOR_PARAM=0`.

    If using a custom mixer, set `USE_MOTOR_PARAM=1` **only** if the mixer was designed with motor parameters.

!!! danger

    We recommend flying your firmware in simulation _before_ loading the firmware onto real hardware to make sure everything is working.

!!! Warning

    It is not recommended to use a _canned mixer for the primary mixer_ and a _custom mixer for the secondary mixer_ **when the secondary mixer needs `USE_MOTOR_PARAM=1`.**
    In other words, both `PRIMARY_MIXER` and `SECONDARY_MIXER` should use motor parameters, or neither should.

    This is important because the canned mixers make assumptions that affect the gains of the controller on the aircraft.
    This means that a canned mixer will require slightly different tuning than a custom mixer might.

### Defining a Custom Mixer

A custom mixer can be defined by:

1. Set `PRIMARY_MIXER` (required) and/or `SECONDARY_MIXER` (optional) to the custom mixer value in the mixer table
2. Load the mixing matrix parameters for either/both the primary or the secondary mixer

The firmware loads a custom mixer by loading all mixing matrix values from parameters.
Since there are 6 inputs to the mixer (\(F_x,F_y,F_z,Q_x,Q_y,Q_z\)) and 10 possible outputs, the mixer is a 6x10 matrix and there are 60 parameters associated with each custom mixer.
For a standard quadrotor, however, most of these would be zero, since only the first 4 outputs (columns of the mixer matrix) would be used.

In addition to the parameters associated with the 6x10 mixing matrix, the [mixer header values](#rosflight-mixer-implementation-details) need to be specified.
Specifically, make sure to define and load the `PRI_MIXER_OUT_i` and the `PRI_MIXER_PWM_i` parameters, which define the output type and the default PWM rate, respectively, for each `i`th output.
See the [Parameter Configuration Page](../hardware-and-rosflight/parameter-configuration.md) for more information on these parameters.

!!! note

    The PWM rate is typically 490 or 50 Hz.

!!! warning "Mixing Matrix PWM Header"

    PWM outputs are run off of hardware timers, which are often tied to particular clocks.
    Since there are typically more PWM channels than clocks (and therefore hardware timers), the PWM rate cannot be set individually for each channel.
    Instead, there are groups of PWM channels that must all be configured to the same rate.

    This is usually only an issue when dealing with multiple types of actuators (e.g. servos and brushless motors) on a given airframe where the actuators require different PWM rates.
    In many cases, servos and motors can be configured to have the same 50Hz rate.
    This is how the ROSflight fixedwing canned mixers do it.

    The number of timers varies based on hardware, so double check your board documentation when creating the custom mixer PWM header.
    If you don't want to worry about it, just set all the `PRI_MIXER_PWM_i` to the same value, if possible.

The recommended way to load a custom mixer is to first compute all the required parameters and save them to a file on the companion computer.
The parameters are named `PRI_MIXER_i_j` or `SEC_MIXER_i_j`, where `(i,j)` is the index of the parameter in the 6x10 mixing matrix.
See the [Parameter Configuration Page](../hardware-and-rosflight/parameter-configuration.md) for more information on these parameters.

A convenience script is available in the `roscopter` ROS2 package that will compute the custom mixer and save the parameter values in a format ready to load.
The convenience script is found [here](https://github.com/rosflight/roscopter/blob/main/roscopter/scripts/compute_multirotor_mixing_matrix.py) with the configuration file found [here](https://github.com/rosflight/roscopter/blob/main/roscopter/config/multirotor_frame_config.yaml).

Once the parameters are saved to a file, load them with the ROS2 service call (make sure `rosflight_io` is running):
```ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{file: absolute/or/relative/path/to/saved/param/file.yaml}"```

Also make sure to save those parameters to memory with the ROS2 service call:
```ros2 service call /param_write std_srvs/srv/Trigger```
