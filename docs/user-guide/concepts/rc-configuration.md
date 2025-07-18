# RC Configuration

## RC Safety Pilot

ROSflight is designed for use with offboard control from experimental and research code.
As such, it provides several mechanisms for an RC safety pilot to intervene if something goes wrong with the control setpoints coming from the companion computer:

  - **RC override switch:** The safety pilot can flip a switch on the transmitter to take back RC control. Attitude and throttle override can be mapped independently, meaning you can choose one or the other, put them on separate switches, or put them both on the same switch.
  - **Stick deviations:** If a stick is deviated from its center position, then that channel is overridden by RC control. This allows the safety pilot to take control without flipping a switch. This may be useful to provide a momentary correction on a single axis. The fraction of stick travel needed to activate the RC override is controlled by the `RC_OVRD_DEV` parameter. The `OVRD_LAG_TIME` parameter controls the amount of time that the override remains active after the sticks return to center.
  - **Minimum throttle:** By default, the flight controller takes the minimum of the two throttle commands from RC and offboard control setpoints. This allows the safety pilot to drop the throttle quickly if needed. This behavior can be turned on or off with the `MIN_THROTTLE` parameter.

## Arming, Errors & Failsafe

The flight controller can only be armed and disarmed via RC control.
Two mechanisms are provided: sticks (left stick down and right to arm, down and left to disarm) and switch.
Only one of these options can be active at a time.

The firmware runs a number of error checks before allowing the flight controller to arm.
Completing the configuration checklist on the [Getting Started](getting-started.md) page should avoid these errors.
In addition to a few internal health checks, the following conditions are checked:

  - **Mixer:** Valid mixer must have been selected (see the [Hardware Setup](hardware-setup.md) documentation page)
  - **IMU calibration:** The IMU must have been calibrated since firmware was flashed (it is recommended that you recalibrate often)
  - **RC:** There must be an active RC connection

In addition to the error checking before arming, the flight controller enters a failsafe mode if the RC connection is lost during flight while armed.
While in failsafe mode the flight controller commands level flight with the throttle value defined by the `FAILSAFE_THR` parameter.

The following is a simplified version of the finite state machine that defines logic used for the arming, error checks, and failsafe operations:

![Arming FSM](../images/arming-fsm-simplified.svg)

The state manager also includes functionality for recovering from hard faults if one were to occur, although this is unlikely with unmodified firmware. If a hard fault occurs while the flight controller is armed, the firmware has the ability to immediately rearm after rebooting to enable continued RC control of the vehicle for recovery.

## Binding your Transmitter to your Receiver

Follow the instructions in your user manual to bind your transmitter to your RC receiver. You may also be able to find a guide on YouTube with instructions; just search for your particular transmitter and receiver model.

## RC Transmitter Calibration

To avoid confusion and to reduce code complexity in the firmware source code, ROSflight does not perform software calibration of RC transmitters. This means that RC calibration must be done on the transmitter itself, as opposed to in software. This is pretty straight-forward for most modern transmitters.

### Configure the full stick output for each channel

The easiest way to do this is to enter the "Servo Setup" Menu (for Spektrum transmitters) and change the servo travel variable. You can watch the raw RC readings from the flight controller by echoing the rc_raw topic from `rosflight_io`

```
ros2 topic echo /rc_raw
```

* Center both sticks on your transmitter
* Apply subtrim until the first four channels all read 1500 exactly (or as close as possible--some RC receivers are worse than others and cannot exactly output 1500 us)
* Set the channel endpoints so that maximum stick deflections result in readings of 1000 and 2000 us.

### Configure stick directions for roll, pitch, and yaw channels.

You now have to make sure your RC transmitter is sending commands consistent with the north-east-down (NED) frame assumed by ROSflight.

You may find this graphic helpful. It shows all the basic stick positions, and the associated output from the first four channels when looking at a raw AETR (aileron, elevator, throttle, rudder) RC signal from `rosflight_io`. Make sure that the stick output is in the correct direction.

![stick_image](../images/sticks.png)

It should be noted that channel assignment can be modified via the `RC_*_CHN` parameters. So, if you are using something other than AETR assignment, the channel index for each stick may be different, but the direction should be the same.

## Switch Configuration

Switches can be configured for the following functions.
To disable a switch for a specific, default function, set the corresponding parameter to `-1`.
Be sure to check that the switch directions operate as you intend, and reverse them in your transmitter if necessary.

### Safety Pilot Configuration

The `RC_ATT_OVRD_CHN` parameter maps a switch to override attitude commands with RC control.
The `RC_THR_OVRD_CHN` parameter maps a switch to override throttle commands with RC control.
To override both with a single switch, set both parameters to the same value (this is the default behavior).

### Arming

By default, arming is done with the sticks (left stick down and right to arm, down and left to disarm).
To use a switch instead, set the `ARM_CHANNEL` parameter to the desired channel.
Setting an arming switch disables arming with the sticks.

### Flight Mode

If desired, you can map a switch to select between attitude control types (angle and rate) in flight by setting the `RC_ATT_CTRL_CHN` parameter to the desired channel.
This can be useful if, for example, you are learning rate mode but want to be able to switch back to attitude mode to help stabilize the vehicle.
This feature is disabled by default.
