# Overview

## Main Components of ROSflight

ROSflight is intended to be used with both a typical flight controller and a companion Linux computer. Although it can be used with just a flight controller, this setup will not offer most of the advantages of ROSflight.

!!! note
    To avoid confusion, we try to consistently use the following terminology:

      - **Flight controller:** The embedded board that runs the ROSflight firmware and performs I/O with sensors and ESCs
      - **Companion computer:** A Linux computer, running ROS2, that is mounted on the vehicle and has a physical, serial connection with the flight controller
      - **Offboard control (setpoints):** The control setpoints passed from the companion computer to the flight controller. The control is "offboard" from the perspective of the flight controller, even though the computer providing those commands is mounted onboard the vehicle.

The following figure illustrates the interactions between the major components of the system:

![System Components](images/components.svg)

### Firmware

The ROSflight [firmware](https://github.com/rosflight/rosflight_firmware) is the low level microcontroller code that runs on the flight controller. This communicates directly with sensors and actuators and serves as the bridge between hardware and higher level software. The firmware itself is designed to do as little as possible, offloading most of the work to the companion computer.

Although higher level control is offloaded to the companion computer, enough control and functionality is included in the firmware to enable a safety pilot to fly the UAV through any portion of the flight with or without an operating companion computer.

### ROSflight IO

[ROSflight IO](https://github.com/rosflight/rosflight_ros_pkgs) is a ROS2 node that runs on the companion computer that communicates directly with the firmware over a serial connection. This serves as the bridge between the firmware and the rest of ROS2 network.

### ROSplane and ROScopter

[ROSplane](https://github.com/rosflight/rosplane) and [ROScopter](https://github.com/rosflight/roscopter) are ROS2 based autonomy stacks that run on the companion computer and do most of the heavy computation of the autopilot. Each portion of their autonomy stacks are organized into highly modular ROS nodes that can be easily swapped out with custom nodes.

ROSplane and ROScopter are not required for using ROSflight and you could choose to use an entirely different autonomy stack if you so desired.

## RC Safety Pilot

ROSflight is designed for use with offboard control from experimental and research code.
As such, it provides several mechanisms for an RC safety pilot to intervene if something goes wrong with the control setpoints coming from the companion computer:

  - **RC override switch:** The safety pilot can flip a switch on the transmitter to take back RC control. Attitude and throttle override can be mapped independently, meaning you can choose one or the other, put them on separate switches, or put them both on the same switch. Details on these switches are provided on the [RC configuration](rc-configuration.md) page.
  - **Stick deviations:** If a stick is deviated from its center position, then that channel is overridden by RC control. This allows the safety pilot to take control without flipping a switch. This may be useful to provide a momentary correction on a single axis. The fraction of stick travel needed to activate the RC override is controlled by the `RC_OVRD_DEV` parameter. The `OVRD_LAG_TIME` parameter controls the amount of time that the override remains active after the sticks return to center.
  - **Minimum throttle:** By default, the flight controller takes the minimum of the two throttle commands from RC and offboard control setpoints. This allows the safety pilot to drop the throttle quickly if needed. This behavior can be turned on or off with the `MIN_THROTTLE` parameter.

## Arming, Errors & Failsafe

The flight controller can only be armed and disarmed via RC control.
Two mechanisms are provided: sticks (left stick down and right to arm, down and left to disarm) and switch.
Only one of these options can be active at a time.
Details on configuration are given on the [RC configuration](rc-configuration.md) page.

The firmware runs a number of error checks before allowing the flight controller to arm.
Completing the configuration checklist on the [Getting Started](getting-started.md) page should avoid these errors.
In addition to a few internal health checks, the following conditions are checked:

  - **Mixer:** Valid mixer must have been selected (see the [Hardware Setup](hardware-setup.md) documentation page)
  - **IMU calibration:** The IMU must have been calibrated since firmware was flashed (it is recommended that you recalibrate often)
  - **RC:** There must be an active RC connection

In addition to the error checking before arming, the flight controller enters a failsafe mode if the RC connection is lost during flight while armed.
While in failsafe mode the flight controller commands level flight with the throttle value defined by the `FAILSAFE_THR` parameter.

The following is a simplified version of the finite state machine that defines logic used for the arming, error checks, and failsafe operations:

![Arming FSM](images/arming-fsm-simplified.svg)

The state manager also includes functionality for recovering from hard faults if one were to occur, although this is unlikely with unmodified firmware. If a hard fault occurs while the flight controller is armed, the firmware has the ability to immediately rearm after rebooting to enable continued RC control of the vehicle for recovery.

## LEDs

The meaning of the various LEDs is summarized in the following table. The colors of the LEDs may change depending on your specific board:

| LED           | On            | Off              | Slow Blink       | Fast Blink       |
|---------------|---------------|------------------|------------------|------------------|
| Power (Blue)  | Board powered | -                | -                | -                |
| Info (Green)  | RC control    | Offboard control | -                | -                |
| Warning (Red) | Armed         | Disarmed         | Error (disarmed) | Failsafe (armed) |
