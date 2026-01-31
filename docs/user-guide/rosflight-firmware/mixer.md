# Mixer

A controller is responsible for computing commands that drive a system to a given reference.
However, in general the output of a controller is not raw motor commands.
For example, given a quadrotor vehicle, the [ROSflight firmware rate controller](../rosflight-firmware/code-architecture.md#controller) takes in desired angular rates and outputs desired forces and torques that will achieve those rates.
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

!!! note

    The rest of this guide focuses on **using** the ROSflight mixer.
    Please see the [ROSflight 2.0 paper](https://arxiv.org/abs/2510.00995) for more information on the derivation and details of the ROSflight mixer.

    Additionally, this guide tries not to repeat information found in the paper, so we recommend reading through that paper to get an overview of the ROSflight mixer.

## ROSflight mixer implementation details
Currently supported hardware for the ROSflight firmware has a maximum of 10 output channels.
Thus, the ROSflight mixing matrix is a 10x10 matrix.

In addition to storing these 100 values, each ROSflight mixer has a header with the following information:

- PWM rate (in Hz) for each output channel
- [Output type](#defining-a-custom-mixer) (auxiliary, motor, servo, GPIO)

Users usually won't have to adjust these header values, unless you [define a custom mixer](#defining-a-custom-mixer) as described below.

### Size of the mixer matrix, input, and output vectors
Note that $n$ denotes the number of possible output commands and $m$ denotes the size of the input commands, which for ROSflight is 10.
Setting $m=10$ was chosen as the maximum number of mixer channels available in ROSflight.
However, the actual number of possible output commands is dependent on the number of PWM hardware outputs, which is board-specific.

### Mapping RC inputs to the $u$-vector
The input command vector, $u$, in ROSflight is a 10x1 vector.
However, only 4 commands get sent by an RC pilot.
In general, these 4 commands deal with roll, pitch, yaw, and throttle commands.

In ROSflight, we need to know how to map these 4 values to appropriate channels in the $u$ vector.
Additionally, this mapping *must* match the order that these values are expected to be in in the mixer.

Thus, ROSflight makes some assumptions about the channels that the $u$-vector contains.
ROSflight assumes that the first 3 channels of the $u$ vector correspond to the $[F_x, F_y, F_z]$ force commands (or similar--for a fixedwing these would be $[\text{throttle}, 0, 0]$).
Channels 3-6 corespond to $[Q_x, Q_y, Q_z]$ torque commands (corresponding to angular movement commands).

The remaining 4 channels are never muxed by the [command mananger](./code-architecture.md#command-manager) and are interpreted as passthrough commands.

### Mapping motor commands to PWMs
Running the mixer equations results in the vector $\tau$ of motor commands.
The elements of $\tau$ are defined to be in the range $[0,1]$ for motor commands and $[-1,1]$ for servo commands.
After computing these values, ROSflight then maps them up to the standard PWM range, that is $[1000\mu s, 2000 \mu s]$.


## Selecting a primary mixer

A primary mixer must be selected before the firmware will allow arming of the vehicle.
The primary mixer is used by the RC safety pilot and thus must be a mixer that allows the RC safety pilot to control the vehicle.
Specifically, the `PRIMARY_MIXER` parameter must be set to a valid value.

As described in the [ROSflight 2.0 paper](https://arxiv.org/abs/2510.00995), ROSflight offers some pre-computed, "canned" mixers that can be used off the shelf for a variety of common multirotor and fixedwing airframes.
These mixers do not take into account all the parameters of your system (i.e. motor and propeller parameters), but instead are based solely on the geometry of the airframe.
If you want a more accurate mixer, or have easy access to the motor and prop parameters of your system, then we recommend using a [custom mixer](#defining-a-custom-mixer).

The desired mixer must be chosen by [setting the `PRIMARY_MIXER` parameter](../rosflight-firmware/parameter-configuration.md) to one of the following valid values.


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
Since there are 10 inputs to the mixer and 10 possible outputs, the mixer is a 10x10 matrix and there are 100 parameters associated with each custom mixer.
For a standard quadrotor, however, most of these would be zero, since only the first 4 outputs (columns of the mixer matrix) would be used.

In addition to the parameters associated with the 10x10 mixing matrix, the [mixer header values](#rosflight-mixer-implementation-details) need to be specified.
Specifically, make sure to define and load the `PRI_MIXER_OUT_i` and the `PRI_MIXER_PWM_i` parameters, which define the output type and the default PWM rate, respectively, for each `i`th output.

The `PRI_MIXER_OUT_i` values should be set to one of the following values:

| `PRI_MIXER_OUT_i` value | How channel is interpreted by mixer |
| :--- | :--- |
| 0 | Auxiliary |
| 1 | Servo |
| 2 | Motor |
| 3 | GPIO |

This designation is important since in ROSflight, motor output commands are one-sided (ranging from zero to 1) where servos are two-sided (ranging from -1 to 1).
The mixer uses the output channel type designation from the above table to know how to scale an output value (from the output vector $\tau$) to a standard PWM command (between 1000 and 2000$\mu s$).

The auxiliary command types are [described below](#auxiliary-inputs).

See the [Parameter Configuration Page](../rosflight-firmware/parameter-configuration.md) for more information on these parameters.

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
The parameters are named `PRI_MIXER_i_j` or `SEC_MIXER_i_j`, where `(i,j)` is the index of the parameter in the 10x10 mixing matrix.
See the [Parameter Configuration Page](../rosflight-firmware/parameter-configuration.md) for more information on these parameters.

A convenience script is available in the `roscopter` ROS2 package that will compute the custom mixer and save the parameter values in a format ready to load.
The convenience script is found [here](https://github.com/rosflight/roscopter/blob/main/roscopter/scripts/compute_multirotor_mixing_matrix.py) with the configuration file found [here](https://github.com/rosflight/roscopter/blob/main/roscopter/config/multirotor_frame_config.yaml).

Once the parameters are saved to a file, load them with the ROS2 service call (make sure `rosflight_io` is running):
```ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{file: absolute/or/relative/path/to/saved/param/file.yaml}"```

Also make sure to save those parameters to memory with the ROS2 service call:
```ros2 service call /param_write std_srvs/srv/Trigger```

## Saturation
There are often aggressive maneuvers which may cause multiple motor outputs to become saturated.
This is of particular concern in multirotors.

!!! example

    Consider the case of commanding maximum roll while at full throttle in a quadcopter.
    Full throttle, with no other command, would normally cause all four motors to saturate at a maximum PWM command of 2000$\mu$s.
    Mixing in a maximum roll command would cause two of the motors to reduce to 1500$\mu$s, while the other two would go to 2500$\mu$s.

    Because 2500$\mu$s is beyond the maximum limit, the maximum roll command would actually be interpreted as a half-roll command as the difference between the motors on opposite sides of the MAV would be halved.

To increase controllability in these sorts of situations, we take the motor control value from the mixer, before it is shifted up to the standard PWM range, and find the scaling factor which would reduce the maximum motor command to 2000$\mu$s.
We then apply this scaling factor to all other motors, and then shift them up to the standard PWM range of 1000$\mu$s to 2000$\mu$s.

This does not completely solve the problem, because the maximum roll command is still reduced somewhat, but it trades off maximum thrust for some additional controllability.

!!! example
    In the case of a simultaneous maximum roll and maximum throttle, two of the motors will receive a command of 2000$\mu$s, while the other two will receive a command of 1333$\mu$s, as opposed to the 1500$\mu$s without the saturation, and maintains controllability even at high levels of throttle.

!!! warning

    ROSflight expects the elements of $\tau$ to be in the range $[0,1]$ for motors and $[-1,1]$ for servos.
    However, if the mixer values or controller gains have not been set properly, it is possible that the elements of $\tau$ are much larger than 1.

    If you consistently get outputs from the mixer greater than 1, then the saturation methods discussed above **will effectively limit the expressiveness of your control**.

    If ROSflight detects an output element of $\tau$ greater than 2 (before scaling), then it will print a warning message to the screen.
    The number 2 is arbitrary. However, take it as a sign that your mixer or controller gains need to be adjusted to avoid excessive scaling/saturation.

## Auxiliary inputs

ROSflight supports auxiliary inputs, which are defined as inputs that bypass the mixer.
Thus, any output channel in the mixer labeled as `AUX` will not use the mixer.
This can be useful for sending servo commands to landing gears, grasping mechanisms, or other more complicated scenarios.

!!! tip

    To set a channel in a custom mixer as an `AUX` channel (bypassing the mixer), set the `PRI_MIXER_OUT_i` parameter to 0, where `i` is the number of the channel (zero indexed).

    These values are hard-coded for the canned mixers.
