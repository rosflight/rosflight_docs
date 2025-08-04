# ROSplane Controller

## Overview

The ROSplane controller generates the controller commands to achieve the commanded states that the path follower directs.
The controller has the basic function of controlling roll, pitch, yaw, airspeed, course and altitude.
Higher functions of controlling position and tracking paths is handled by other nodes, the path follower and manager specifically.
To gain the best understanding of the Controller and its role, read chapter 1 of the UAV book, or the ROSplane Overview page.

<!-- TODO add glam GIF or pic of response to show how awesome ROSplane is  -->

## Interfaces

| ![Diagram of Controller ROS Interactions](../../../assets/controller_assets/Controller_ROS.png "Controller ROS Interactions") |
|:--:|
|*Figure 1: Controller's ROS interactions.*|

### Input

The controller receives controller commands using the ControllerCommands message on the `\controller_commands` topic from the `path_follower` node. 
This set of commands are outlined below, along with a short explanation of each.

| Message Field | Explanation | Units |
|:------:|:-------:| :---: |
| header | This contains time stamp information. | Time in Seconds and Nanoseconds (int) |
| va_c | The commanded airspeed. | Meters per second (float) |
| h_c | The commanded altitude. | Meters (float) |
| chi_c | The commanded course. | Radians (float) |
| phi_ff | Feedforward roll term (for orbits) | Radians (float) |
| aux[4] | Four array of auxiliary commands | None (float) |
| aux_valid | Indicates whether aux vector contains actual information. | True/False (bool) |

The controller uses these targets to control the aircraft.
This drives the aircraft to approach and follow the path as the path_follower commands.
See the Path Follower page for more information.
See the Successive Loop Closure Controller Outline for more information on how these commands are used specifically.

!!! note 
    Poor path performance may be due to controller or the path follower, see the Tuning Guide in the User Guide section for more details.

### Output

The controller calculates the control surface outputs in percent deflection (based around zero) and throttle.
It formats and the publishes these outputs in the Command message on the `/command` topic.
There are a further four auxiliary channels that can be used in the Command message, however they are unused typically.
A summary of the parts of the command message are as follows.

| Message Field | Explanation | Units |
|:------:|:-------:| :---: |
| header | This contains time stamp information. | Time in Seconds and Nanoseconds (int) |
| mode | The control mode (used in multi-rotors) | None (int) |
| ignore | A bitmask to ignore particular values. | None (int) |
| x | Aileron Command | Percent deflection in direction [-1.0, 1.0] (float) |
| y | Elevator Command | Percent deflection in direction [-1.0, 1.0] (float) |
| z | Rudder Command | Percent deflection in direction [-1.0, 1.0] (float) |
| f | Throttle Command | Percent of full throttle [0.0, 1.0] (float) |

These are passed to `rosflight_io`, which formats them into MAVLink messages and forwards them onto the FCU.
!!! note 
    For this to work the parameters on the `rosflight_firmware` must be set to work for an airplane.
    See the User Guide on first time start up or README.md for the repository for more details on firmware setup.

## Running the Controller

As mentioned in the ROSplane ROS overview, the controller is in the main `rosplane` ROS package.
The ROS executable is `controller`, yielding the run command:

`ros2 run rosplane controller`

The type of controller used is passed right after the executable name, substitute `control_type` for controller being used.

`ros2 run rosplane controller control_type`

To pass a set of parameters for the controller from a yaml file using the `--ros-args` option.

`ros2 run rosplane controller --ros-args --params-file path/to/params.yaml`

Putting it all together,

`ros2 run rosplane controller control_type --ros-args --params-file path/to/params.yaml`

A table of arguments and parameter files that work out of the box is given in the following table.
!!! note 
    Filepaths will need to be altered to work.
!!! note
    The most common way of running the controller is through a launch file with the rest of the ROSplane pipeline running as well.
    See the ROSplane Overview in the Developer and User Guides for more details.


| Argument | Explanation | Values |
|:------:|:-------:| :---: |
| --params-file | The parameters file that contains the gains and other important parameters. | `rosflight_ws/src/rosplane/rosplane/params/anaconda_autopilot_params.yaml`,`rosflight_ws/src/rosplane/rosplane/params/skyhunter_autopilot_params.yaml` |
| control_type | These arguements are passed directly to the main function defined in the controller. | `default`, `total_energy` |
