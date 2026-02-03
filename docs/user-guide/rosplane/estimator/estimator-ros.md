# Estimator ROS

## Overview

The estimator ROS class implements the basic ROS interfaces for the estimator.
This includes setting up subscribers, publishers and initializing parameter management.
The idea of the ROS class, is that all interfacing with ROS and shared resources across all inheritance levels happens or are contained in this class.

## ROS Interfaces

The estimator has the following ROS interfaces.
The estimator has several ROS interfaces that are tracked as member variables of the `estimator_ros` class.
They are summarized in the table below:

| ROS Interface | Topic | Explanation | Message Type |
|:------:|:-------:| :---: | :---: |
| <div style="white-space: nowrap;">`vehicle_state_pub_`<div> | `/estimated_state` | Publishes the estimated state of the vehicle. | rosplane_msgs/msg/State |
| <div style="white-space: nowrap;">`gnss_sub_`<div> | `/gnss` | Subcribes to GNSS position and velocity. | rosflight_msgs/msg/GNSS |
| <div style="white-space: nowrap;">`imu_sub_`<div> | `/imu/data` | Subcribes to the IMU data (both gyro and accel). | sensor_msgs/msg/Imu |
| <div style="white-space: nowrap;">`baro_sub_`<div> | `/baro` | Subcribes to the barometer pressure information. | rosflight_msgs/msg/Barometer |
| <div style="white-space: nowrap;">`airspeed_sub_`<div> | `/airspeed` | Subcribes to the differential pressure information. | rosflight_msgs/msg/Airspeed |
| <div style="white-space: nowrap;">`status_sub_`<div> | `/status` | Subcribes to the aircraft status information. | rosflight_msgs/msg/Status |
| <div style="white-space: nowrap;">`magnetometer_sub_`<div> | `/magnetometer` | Subcribes to the magnetometer field vector. | sensor_msgs/msg/MagneticField |

!!! note 
    The IMU and magnetometer messages are from standard ROS message packages. GNSS, barometer, airspeed, and status messages are from `rosflight_msgs`.

## Parameters

| **Parameter** | **Explanation** | **Type** | **Range** |
| :---: | :---: | :---: | :---: |
| `estimator_update_frequency` | The frequency that the estimator will run estimations. | double | 390.0 |
| `rho` | The density of the air. Typically calculated via the 1976 Standard atmosphere model using the GNSS altitude when `NOT_IN_USE`. | double | NOT_IN_USE $\frac{kg}{m^3}$ |
| `gravity` | The acceleration due to gravity. | double | 9.81 $\frac{m}{s^2}$ |
| `gps_ground_speed_threshold` | Minimum velocity to consider course calculation to be valid. | double | 0.3 $\frac{m}{s}$ |
| `baro_measurement_gate` | Maximum altitude that can be added in a single barometer update. | double | 1.35 |
| `airspeed_measurement_gate` | Maximum jump in pascal difference we can tolerate. | double | 5.0 |
| `baro_calibration_count` | Num samples to use in calibration calculation. | int | 100 |
| `max_imu_sensor_silence_duration_ms` | Max IMU sensor silence duration. | int | 4 |
| `max_mag_sensor_silence_duration_ms` | Max magnetometer sensor silence duration. | int | 25 |
| `max_baro_sensor_silence_duration_ms` | Max barometer sensor silence duration. | int | 15 |
| `max_gnss_sensor_silence_duration_ms` | Max GNSS sensor silence duration. | int | 110 |
| `max_diff_sensor_silence_duration_ms` | Max differential pressure sensor silence duration. | int | 110 |
| `min_gnss_fix_type` | Minimum GNSS fix type (float). | int | 3 |
| `hotstart_estimator` | Whether the estimator should use preset hotstart values. | bool | false |

## Modifying the Estimator

The ROS class can be overridden in two basic ways,

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
