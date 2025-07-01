# Pre-Flight Checklist

This is an example of a ROSflight pre-flight checklist. You will likely need to augment this with checks specific to both (a) your hardware and (b) the code running on your companion computer.

## Before powering up motors
- [ ] ROS2 is running on the companion computer, communicating with the base station
- [ ] `rosflight_io` reports no errors
- [ ] Sensors are calibrated and publishing
    + [ ] IMU (re-calibrate every flight): `ros2 service call /calibrate_imu std_srvs/srv/Trigger`
    + [ ] Barometer: `ros2 service call /calibrate_baro std_srvs/srv/Trigger`
    + [ ] Sonar (if attached)
    + [ ] Airspeed (if attached)
- [ ] Estimated attitude is being published and looks accurate
- [ ] Published outputs look reasonable
- [ ] Parameter Check (if using a fixed-wing, there are about 8 parameters you will need to change from default)
- [ ] RC communication
- [ ] Failsafe behavior
- [ ] Arming and disarming
- [ ] RC override behavior
- [ ] RC range test
- [ ] Wire-wiggle test (wiggle all wires to look for bad connections)
- [ ] If desired, logging is turned on (e.g. recording a ros2 bag)

## After Powering Up Motors

!!! danger
    Be sure the flight controller is disarmed before powering up motors!!!
- [ ] Arm/Disarm test
- [ ] Propeller spin test (check directions and response to stick inputs)
- [ ] Control surface test (fixed-wing)
- [ ] Response to offboard controls
