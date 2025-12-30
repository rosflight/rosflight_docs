# Estimator ROS

## Overview

The estimator base implements the basic ROS interfaces for the estimator.
This includes setting up subscribers, publishers and initializing parameter management.
The idea of the base class, is that all interfacing with ROS and shared resources across all inheritance levels happens or are contained in this class.

## ROS Interfaces

The estimator has the following ROS interfaces.
The estimator has several ROS interfaces that are tracked as member variables of the `estimator_base` class.
They are summarized in the table below:

| ROS Interface | Topic | Explanation | Message Type |
|:------:|:-------:| :---: | :---: |
| <div style="white-space: nowrap;">`vehicle_state_pub_`<div> | `/estimated_state` | Publishes the estimated state of the vehicle. | State.msg |
| <div style="white-space: nowrap;">`gnss_fix_sub_`<div> | `/navsat_compat/fix` | Subcribes to the GNSS position information. | NavSatFix.msg |
| <div style="white-space: nowrap;">`gnss_vel_sub_`<div> | `/navsat_compat/vel` | Subcribes to the GNSS velocity information. | TwistStamped.msg |
| <div style="white-space: nowrap;">`imu_sub_`<div> | `/imu/data` | Subcribes to the IMU data (both Gyro and Accel). | Imu.msg |
| <div style="white-space: nowrap;">`baro_sub_`<div> | `/baro` | Subcribes to the barometer pressure information. | Barometer.msg |
| <div style="white-space: nowrap;">`airspeed_sub_`<div> | `/airspeed` | Subcribes to the differential pressure information. | Airspeed.msg |
| <div style="white-space: nowrap;">`status_sub_`<div> | `/status` | Subcribes to the aircraft status information. | Status.msg |

!!! note 
    The Imu message is from standard ROS message packages. Barometer, Airspeed and Status messages are from `rosflight_msgs`.

## Parameters

| **Parameter** | **Explanation** | **Type** | **Range** |
| :---: | :---: | :---: | :---: |
| `rho` | The density of the air. Optional, will calculate using the 1976 Atmospheric model using alt otherwise. | double | ~1.225 $\frac{kg}{m^3}$ |
| `gravity` | The acceleration due to gravity. | double | ~9.81 $\frac{m}{s^2}$ |
| `estimator_update_frequency` | The frequency that the estimator will run estimations. | double | $\geq 100$ Hz |
| `gps_ground_speed_threshold` | This determines when the aircraft is moving fast enough to use GNSS velocity measurements to calculate a course angle. | double | $\geq$ 0.3 $\frac{m}{s}$ |
| `baro_measurement_gate` | The maximum allowable instantaneous change in barometer pressure measurement. | double | ~1 meter |
| `airspeed_measurement_gate` | The maximum allowable instantaneous change in differential pressure measurement. | double | ~5 $\frac{m}{s}$ |
| `baro_calibration_count` | The number of barometer measurements used to do calibration. This number is more or less arbitrary. | int | $\geq 100$ |

## Modifying the Estimator

The base class can be preempted in two basic was,

1. Complete removal,
2. Inheritance from `EstimatorROS`,

Each of these methods has pros and cons and more decisions that need to be made.
Following this is a brief overview of these options.

### Complete removal

Because ROSplane takes full advatage of ROS, as long as there is a topic `/estimated_state` using the `rosplane_msgs/msg/State` is being published, the rest of ROSplane will operate as expected.
This means that the entirety of the estimator module can be removed and replace with a ROS node of your design as long as it publishes the appropiate information.
To emphasize, `rosflight_io` will not check that its sensor topics are being received, the other nodes in ROSplane similarly will not interrogate the estimator in any other way than through the `/estimated_state` topic.
The system is truly modular.

This option would likely be the most attractive if you are adapting an existing ROS package or if a significant number of other sensors will be used.
It would also be a good option if the estimator in use is written in Python or another language other than C++.
It could also be a good option if the estimator was developed in a completely different environment.

### Inheritance from `EstimatorROS`

Inheriting from `EstimatorROS` takes care of all the ROS interfacing for you.
This option would likely be the most attractive to those who do not mind adapting their existing estimator to interface correctly with ROSplane, or those developing the estimator directly using ROSplane but not using an EKF.
The inheritance can be done further down the inheritance chain, see the Estimator EKF page for details.

