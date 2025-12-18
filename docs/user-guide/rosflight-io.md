# ROSflight IO

As seen in the diagram below, the `rosflight_io` node is one of the major components in the ROSflight ecosystem.
This document describes the responsibilities of the `rosflight_io` node and how users should expect to interact with the node.

| ![System Components](images/components.svg) |
| :---: |
| *Diagram of the major system components in ROSflight* |


The `rosflight_io` node is a ROS 2 node that runs on the companion computer.
Code for this node is located in the [`rosflight_ros_pkgs`](https://github.com/rosflight/rosflight_ros_pkgs.git) repository on GitHub.
This code can be installed and run following the [installation](./installation/installation-sim.md) and [tutorial](./tutorials/manually-flying-rosflight-sim.md) guides.

!!! note

    The `rosflight_io` node is technically needed only if communication between the companion computer and the FCU is needed.
    In nearly all cases, however, communication is needed.

    The `rosflight_io` node provides telemetry and status updates, even when only flying manually (no companion computer control).

## Responsibility
The responsibility of the `rosflight_io` node is to handle communication between the companion computer and the flight control unit (FCU), which runs the ROSflight firmware.
This includes

- Sensor information (FCU &rarr; companion computer)
- Status and log messages (FCU &rarr; companion computer)
- Actuator or controller commands (companion computer &rarr; FCU)

## Running `rosflight_io`
The `rosflight_io` node is one of the few nodes in the ROSflight ecosystem that "knows" if it is simulation or hardware.
It needs to know this because it manages the serial connection between the FCU and the companion computer.

To run `rosflight_io` when physically connected via serial to the FCU, run
```bash
# Replace /dev/ttyACM0 with the location of the serial device on your machine
ros2 run rosflight_io rosflight_io --ros-args -p port:=/dev/ttyACM0
```

When running in simulation, the `rosflight_io` bounces the communication against a UDP connection, thus simulating the serial connection between the FCU and companion computer.
Thus, to run `rosflight_io` in simulation run
```bash
ros2 run rosflight_io rosflight_io --ros-args -p udp:=true
```

!!! note
    Both the UDP and the serial connection parameters can be configured as described in the [Parameters and Configuration](#parameters-and-configuration) section below.

!!! tip

    The `rosflight_io` node prints all the logging messages it receives to the terminal where it was launched.
    It can be very useful to keep an eye on these messages---this is often the first indication of when something goes wrong.

When `rosflight_io` is launched, it

1. Initializes connection with the FCU (simulated or not)
2. Requests all parameters from the FCU
3. Requests the ROSflight version number from the FCU
4. Starts sending heartbeat messages to the FCU

On a successful connection in simulation, `rosflight_io` should output something like
```bash
[INFO] [1766001828.320478247] [rosflight_io]: Connecting over UDP to "localhost:14525", from "localhost:14520"
[WARN] [1766001828.340941543] [rosflight_io]: RC override active
[WARN] [1766001828.341085537] [rosflight_io]: Autopilot now in ANGLE mode
[WARN] [1766001828.346737624] [rosflight_io]: ROSflight version does not match firmware version. Errors or missing features may result
[WARN] [1766001828.346814941] [rosflight_io]: ROSflight version: 2.0
[WARN] [1766001828.346847340] [rosflight_io]: Firmware version: undefined
[INFO] [1766001828.441092908] [rosflight_io]: Got HEARTBEAT, connected.
[INFO] [1766001828.443783649] [rosflight_io]: Detected time offset of 1766001772.428 s.
[INFO] [1766001829.353758804] [rosflight_io]: [Autopilot]: Loading saved custom values to primary mixer...
[INFO] [1766001829.356130174] [rosflight_io]: [Autopilot]: Invalid mixer selected for secondary mixer!
[INFO] [1766001829.356228048] [rosflight_io]: [Autopilot]: Secondary mixer defaulting to primary!
[INFO] [1766001829.358746094] [rosflight_io]: [Autopilot]: ARM switch mapped to RC channel 4
[INFO] [1766001829.358793040] [rosflight_io]: [Autopilot]: ATTITUDE OVERRIDE switch mapped to RC channel 5
[INFO] [1766001829.361003054] [rosflight_io]: [Autopilot]: THROTTLE OVERRIDE switch mapped to RC channel 5
[INFO] [1766001829.361065877] [rosflight_io]: [Autopilot]: ATTITUDE TYPE switch mapped to RC channel 6
[INFO] [1766001831.321671676] [rosflight_io]: Received all parameters
```

Most of these messages are status or logging messages sent from the FCU to `rosflight_io` for various startup checks inside the firmware.

!!! tip

    The key message in the above output is the line
    ```bash
    [INFO] [1766001831.321671676] [rosflight_io]: Received all parameters
    ```
    This typically means that `rosflight_io` and the FCU booted up and are communicating correctly.

## Serial communication between the FCU and the companion computer
The FCU and `rosflight_io` need to be able to communicate over the serial connection.
This serial communication is currently implemented using [MAVlink](https://mavlink.io/en/).
Thus, one of the core responsibilities of the `rosflight_io` node is to convert ROS 2 messages to MAVlink messages and vice versa.

Note that other serial communication protocols could be implemented, or even a system such as [microROS](https://micro.ros.org/) could be used.

## Using `rosflight_io`/implementation details
Users interact with the `rosflight_io` node using ROS 2 interfaces, namely publishers, subscribers, and services.

| Subscriber name | Message type | Description |
| :--- | :--- | :--- |
| `aux_command` | `rosflight_msgs/AuxCommand` | Auxiliary commands to send to the FCU (aux commands are not routed through the firmware's mixer) |
| `command` | `rosflight_msgs/Command` | Command setpoints from the companion computer to send to the FCU (command setpoints are routed through the firmware's mixer) |
| `external_attitude` | `rosflight_msgs/Attitude` | Attitude estimates derived from some onboard estimator that are used to supplement the FCU's complementary filter |

| Publisher name | Message type | Description |
| :--- | :--- | :--- |
| `airspeed` | `rosflight_msgs/Airspeed` | Airspeed sensor data |
| `attitude` | `rosflight_msgs/Attitude` | FCU complementary filter attitude estimate |
| `attitude/euler` | `geometry_msgs/Vector3Stamped` | FCU complementary filter attitude estimate (using Euler angles) |
| `baro` | `rosflight_msgs/Barometer` | Barometer sensor data |
| `battery` | `rosflight_msgs/BatteryStatus` | Battery sensor data |
| `gnss` | `rosflight_msgs/GNSS` | GNSS sensor data |
| `imu/data` | `sensor_msgs/Imu` | IMU sensor data |
| `imu/temperature` | `sensor_msgs/Temperature` | IMU temperature data |
| `magnetometer` | `sensor_msgs/MagneticField` | Magnetometer data |
| `output_raw` | `rosflight_msgs/OutputRaw` | Raw FCU PWM output |
| `rc_raw` | `rosflight_msgs/RCRaw` | Raw RC input received by the FCU |
| `sonar` | `sensor_msgs/Range` | Range sensor data |
| `status` | `rosflight_msgs/Status` | FCU status messages |
| `status/params_changed` | `std_msgs/Bool` | Flag indicating that the FCU's parameters have changed |
| `status/rosflight_errors` | `rosflight_msgs/Error` | Bitfield indicating the FCU's error status |
| `status/unsaved_params` | `std_msgs/Bool` | Flag indicating if the FCU has unsaved parameters |
| `version` | `std_msgs/String` | FCU's firmware version number |

!!! note

    The `rosflight_io` node creates sensor publishers when the sensor message is first received from the firmware.
    If you don't see a particular sensor publisher, it means that the firmware is not correctly sending that sensor information over the serial connection.

### Service servers
The main way to interact with the FCU (running the ROSflight firmware) is through the service servers advertised by `rosflight_io`.

The `rosflight_io` node offers the following service servers:

| Service name | Interface type | Description |
| :--- | :--- | :--- |
| `all_params_received` | `std_srvs/Trigger` | Returns true if all parameters have been received by `rosflight_io` |
| `calibrate_airspeed` | `std_srvs/Trigger` | Instructs the firmware to calibrate the airspeed sensor |
| `calibrate_baro` | `std_srvs/Trigger` | Instructs the firmware to calibrate the barometer |
| `calibrate_imu` | `std_srvs/Trigger` | Instructs the firmware to calibrate the IMU |
| `calibrate_mag` | `std_srvs/Trigger` | Instructs the firmware to calibrate the magnetometer |
| `calibrate_rc_trim` | `std_srvs/Trigger` | Instructs the firmware to calibrate the RC trim values |
| `param_get` | `rosflight_msgs/ParamGet` | Gets a parameter from the firmware |
| `param_load_from_file` | `rosflight_msgs/ParamFile` | Sequentially loads each parameter in a parameter file to the firmware |
| `param_save_to_file` | `rosflight_msgs/ParamFile` | Saves the current firmware parameters to a file |
| `param_set` | `rosflight_msgs/ParamSet` | Sets an individual parameter in the firmware |
| `param_write` | `std_srvs/Trigger` | Instructs the firmware to write all parameters to memory |
| `reboot` | `std_srvs/Trigger` | Instructs the firmware to reboot |
| `reboot_to_bootloader` | `std_srvs/Trigger` | Instructs the firmware to reboot to bootloader |


??? example "Example - Calibrating the IMU"

    To calibrate the IMU, run
    ```bash
    ros2 service call /calibrate_imu std_srvs/srv/Trigger
    ```

    And after a bit you should see output like
    ```bash
    [INFO] [1766004526.090247047] [rosflight_io]: Parameter GYRO_X_BIAS has new value 0
    [WARN] [1766004526.090315867] [rosflight_io]: There are unsaved changes to onboard parameters
    [INFO] [1766004526.090336231] [rosflight_io]: Parameter GYRO_Y_BIAS has new value 0
    [INFO] [1766004526.090360339] [rosflight_io]: Parameter GYRO_Z_BIAS has new value 0
    [INFO] [1766004526.090381384] [rosflight_io]: Parameter ACC_X_BIAS has new value 0
    [INFO] [1766004526.090402445] [rosflight_io]: Parameter ACC_Y_BIAS has new value 0
    [INFO] [1766004526.090431170] [rosflight_io]: Parameter ACC_Z_BIAS has new value 0
    [INFO] [1766004528.592684699] [rosflight_io]: Parameter ACC_X_BIAS has new value -0.0493735
    [WARN] [1766004528.592814409] [rosflight_io]: There are unsaved changes to onboard parameters
    [INFO] [1766004528.592847257] [rosflight_io]: Parameter ACC_Y_BIAS has new value 0.214831
    [INFO] [1766004528.592960904] [rosflight_io]: Parameter ACC_Z_BIAS has new value 0.0190079
    [INFO] [1766004528.592987462] [rosflight_io]: [Autopilot]: IMU offsets captured
    [INFO] [1766004528.593018820] [rosflight_io]: Parameter GYRO_X_BIAS has new value -0.181556
    [INFO] [1766004528.593048466] [rosflight_io]: Parameter GYRO_Y_BIAS has new value -0.142443
    [INFO] [1766004528.593073161] [rosflight_io]: Parameter GYRO_Z_BIAS has new value 0.218234
    ```

## Parameters and configuration
The `rosflight_io` node has the following parameters associated with it

| Parameter name | Parameter type | Description |
| :--- | :--- | :--- |
| `udp` | `bool` | Whether or not to use a UDP connection instead of the serial device |
| `bind_host` | `string` | IP address of the host computer (the one running `rosflight_io`)  |
| `bind_port` | `int` | Port on the host computer (the one running `rosflight_io`) to use  |
| `remote_host` | `string` | IP address of the remote computer (the one running the simulated board)  |
| `remote_port` | `int` | Port on the remote computer (the one running the simulated board) to use  |
| `port` | `string` | Location of the serial device (e.g. `/dev/ttyACM0`) |
| `baud_rate` | `int` | Baud rate for the serial connection |
| `frame_id` | `string` | Frame ID for sensor messages (like IMU/magnetometer, etc.) |

Note that if `udp=true`, then `rosflight_io` will default to a UDP connection and will not connect to a serial device located at the value of the `port` parameter.

### Convenience parameters
Changing the FCU's parameters via the `param_get` and `param_set` service calls is slow, and can be annoying if you are trying to tune firmware parameters rapidly.
To help with this, `rosflight_io` exposes some parameters to the ROS 2 parameter system and manages setting and syncing the values of those parameters with the firmware's values.

By default, only the firmware controller's PID gains are exposed, but others could be exposed as well.
See the [`rosflight_io` code](https://github.com/rosflight/rosflight_ros_pkgs) for details.

These parameters can be set using the ROS 2 parameter tools via the command line using 
```bash
ros2 param set /rosflight_io <PARAM_NAME>
```
or by using RQT's dynamics reconfigure plugin with
```bash
rqt --standalone rqt_reconfigure
```
