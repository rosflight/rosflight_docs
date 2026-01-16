# Estimator Overview

## Overview

The estimator is a continuous-discrete full state Kalman filter, a full treatment of the filter is found in section 8.11 of the [UAV book](https://github.com/randybeard/mavsim_public).
Both ROSplane and ROScopter utilize the same estimator with the minor difference that there is not differential pressure sensor and wind is not estimated in the case of ROScopter. 
This page will outline the states, their meaning and any general notes on the states.
For a more in depth look at which states are estimated in which way visit the [Estimator ROS](./estimator-ros.md) and [Estimator Continuous-Discrete](./estimator-continuous-discrete.md) pages.

## ROS Interactions

The estimator takes in sensor information from `rosflight_io` computes an estimate and publishes it to the rest of ROSplane.


| ![Diagram of Estimator ROS Interactions](../../../assets/estimator_assets/estimator_full_ros_interactions.png "Diagram of Estimator ROS Interactions") |
|:--:|
|*Figure 1: ROS network interactions for the estimator.*|

### Input

The inputs to the estimator are, accelerometer, rate gyro, barometer, differential pressure, magnetometer, GPS position, and GPS velocity estimates.
The table with the topic for each of the measures is below.

| Measure | Explanation | Topic |
|:------:|:-------:| :---: |
| Accelerometer | This measures the specfic force applied to the aircraft in the body frame axes (see section 7.1 in the UAV book for more details). | `/imu/data/linear_acceleration` |
| Rate Gyro | This measures the angular velocity of the aircraft around the body frame axes. | `/imu/data/angular_velocity` |
| Barometer | The barometer measures the ambient air pressure. It is calibrated on arm to establish a "zero" altitude measurement. | `/baro/pressure` |
| Differential Pressure | The differential pressure sensor, measures the difference in pressure using a pitot tube due to forward velocity. | `/airspeed/differential_pressure` |
| Magnetometer | Measures the intesity of the Earth's magnetic field. | `/magnetometer` |
| GNSS Position | GNSS postion gives the position of the aircraft in latitude, longitude and altitude. | `/gnss` |
| GNSS Velocity | GNSS velocity gives the velocity of the aircraft in meters per second in the global NED frame. | `/gnss` |
| Status | Indicates whether the aircraft is armed (indicating a need to initialize position and altitude estimates). | `/status` |

These topics provide the measures that are fused to create a state estimate.

## Output

Below is a table of the `/estimated_state` message and what each of the fields represents.

| Field | Explanation | Type/Units |
|:--:|:--:|:--:|
| header | Standard ROS header. | std_msgs/Header |
| p_n | North position. | float32 (m) |
| p_e | East position. | float32 (m) |
| p_d | Down position. | float32 (m) |
| v_x | Inertial north velocity (u), expressed in body frame. | float32 (m/s) |
| v_y | Inertial east velocity (v), expressed in body frame. | float32 (m/s) |
| v_z | Inertial down velocity (w), expressed in body frame. | float32 (m/s) |
| p | Body frame roll rate. | float32 (rad/s) |
| q | Body frame pitch rate. | float32 (rad/s) |
| r | Body frame yaw rate. | float32 (rad/s) |
| phi | Roll angle. | float32 (rad) |
| theta | Pitch angle. | float32 (rad) |
| psi | Yaw angle. | float32 (rad) |
| b_x | Gyro bias x. | float32 (rad/s) |
| b_y | Gyro bias y. | float32 (rad/s) |
| b_z | Gyro bias z. | float32 (rad/s) |
| quat | Quaternion (wxyz, NED) body to inertial. | geometry_msgs/Quaternion |
| initial_lat | Initial/origin latitude. | float64 (deg) |
| initial_lon | Initial/origin longitude. | float64 (deg) |
| initial_alt | Initial/origin altitude. | float64 (m) |
| wn | Wind north component. | float32 (m/s) |
| we | Wind east component. | float32 (m/s) |
| va | Airspeed. | float32 (m/s) |
| alpha | Angle of attack. | float32 (rad) |
| beta | Side slip angle. | float32 (rad) |
| chi | Course angle. | float32 (rad) |

!!! note 
    More states may be estimated in the estimator than listed, these states are just those useful to other parts of ROSplane.

## Running the Estimator

The estimator is in the main `rosplane` ROS package.
The ROS executable is `estimator`, yielding the run command:

`ros2 run rosplane estimator`

To pass a set of parameters for the controller from a yaml file using the `--ros-args` option.

`ros2 run rosplane estimator --ros-args --params-file path/to/params.yaml`


!!! note
    The most common way of running the estimator is through a launch file with the rest of the ROSplane pipeline running as well.
    See the ROSplane Overview in the Developer and User Guides for more details.
