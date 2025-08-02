# Simulator Architecture

This document describes the architecture of the simulator code.
It describes each node's function and role in the simulator, as well as how different visualizers require different configurations of nodes.

It also details what users would need to do to use their own visualizer with `rosflight_sim`.

!!! note "Prerequisites"

    This guide assumes that you have installed and set up the ROSflightSim, as detailed in [the tutorials](../tutorials/setting-up-rosflight-sim.md).
    

## Big Picture - Sim Architecture

!!! danger "TODO"
    continue here... This page is still under construction. Check back soon!

### Design Philosophy

The design goal of the simulator is to mimic hardware as closely as possible.
This means that the same code that flies on hardware should also be flying the aircraft in sim.
This is a **essential** to ensure that the transition from sim to hardware goes as smoothly as possible.

To that end, each module in the simulator mimics a physical module on the aircraft or in the real world.
For example, the physical sensors are replaced with a "simulated sensors" module, and the physical dynamics are replaced by a "dynamics" module.
Apart from these simulated modules, **the rest of the ROSflight code is the same between hardware and simulation**.

To best mimic the hardware experience of ROSflight, the SIL plugin actually implements the firmware source code as a library.
We just implemented a different "board layer" which uses the `sil_board` functions instead of hardware calls for things like `imu_read()` and `pwm_write()`.
Instead of a serial link over USB to the flight controller, we use a UDP connection bouncing off of localhost to communicate between `rosflight_io` and the firmware.
This means the interface to the SIL plugin is identical to that of hardware.
The `rosflight_io` node is the main gateway to the firmware in simulation, just as it is in hardware.

The following table summarizes the correlation between connections in hardware and simulation:

| Connection Type                         | Hardware     | Simulation                               |
|-----------------------------------------|--------------|------------------------------------------|
| Serial communications to `rosflight_io` | USB / UART   | UDP                                      |
| RC                                      | PPM/SBUS Receiver | ROS2 `RC` topic (`rosflight_msgs/RCRaw`) |
| Motors                                  | PWM          | Handled by `forces_and_moments`           |
| Sensors                                 | SPI/I2C      | `sensors` module                          |


### Architecture

The simulator architecture is diagrammed below.

| ![Simulator Architecture](../images/simulator_architecture.svg) |
| --- |
| Fig 1: Architecture of the simulator. Note that the blue dashed box refers to the only parts that are active when running on hardware, while every module runs in sim. |

Throughout this guide, we will refer to **_modules_** in the simulator.
In Fig 1, each module is represented by a green box, and replaces a process or component present on a real, physical system.
A detailed description of each module is found [below](#module-descriptions).
Each module is **implemented as a separate ROS2 node**, making the simulator more modular and flexible.

The modules communicate with each other via the arrows shown in Fig 1.
Most of the time, these arrows refer to publisher/subscriber interfaces between the nodes.
Other arrows refer to service calls.
Note that not all communication lines are shown.
See the code or use `rqt_graph` for a more complete description of how the simulation nodes communicate with each other.

The dashed black box represents `rosflight_sim`, where the green nodes are all the components of the simulator.
The dashed blue box denotes the modules that are the only nodes present when using ROSflight on real hardware.
In other words, these blue nodes are still used in sim, but the green nodes are not present when using real hardware.

!!! warning "Information separation"

    It is important to note that the blue nodes do not "know" that they are in sim, making a more realistic simulator.
    In other words, the blue nodes depend only on information passed between themselves, so the code is the same in sim as it is on hardware.

    For example, the `rosflight_firmware` box (in blue) is located inside the `rosflight_sim` box, since the `sil_board` node has an instantiation of the `rosflight_firmware` object.
    When the `rosflight_firmware` code calls functions that usually would interact with physical components on hardware, the `sil_board` instead calls the corresponding simulated module.

    One example is the `imu_read` function.
    Normally on hardware, the `imu_read` function reads the IMU data from a buffer that is filled asynchronously over serial by the physical IMU.
    In sim, however, this IMU data is created by the `sensors` module, and is sent to the `sil_board` via a pub/sub interface.
    Then, when the firmware calls `imu_read`, the `sil_board` passes up the simulated information, **in the same way that the physical board would have read the data from the serial buffer**.

    In a similar fashion, when the firmware calls `pwm_write`, instead of writing the PWM command to the servos/ESCs (as is done on hardware), the `sil_board` instead publishes the PWM commands over the `sim/pwm_output` topic to the `forces_and_moments` node.

!!! note "Implications of information separation"

    Separating the flow of information in a realistic manner has some consequences.
    One such consequence in sim is that there is duplicate information flowing over the ROS2 network.

    For example, when the `sensors` module creates IMU data, it is sent over the ROS2 network via a pub/sub interface to the `sil_board`.
    When the firmware reads that information via the `imu_read` function call, it does some processing but ultimately sends that information via MAVlink to `rosflight_io`.
    The `rosflight_io` node then publishes that information on a separate topic to the ROS2 network.
    Thus, **two copies** of the same IMU data are sent across the ROS2 network.

    While having duplicate information is not ideal, it is more important (from our view) that the simulation is realistic--acting the same way as physical hardware, thereby decreasing the cost to transition from sim to hardware.

    Note also that `rosflight_io` publishes the IMU data to the ROS2 network since in hardware, users often need to know or plot that information.

### Flow of information

A single simulation loop starts with the `rosflight_sil_manager` node.
This node calls a service served up by the `sil_board` node, the `tick` service.
This `tick` service corresponds to a single iteration of the main loop in the ROSflight firmware (see the [relevant source code](https://github.com/rosflight/rosflight_firmware/blob/main/src/rosflight.cpp), the `run()` function).

On a `tick`, the `rosflight_firmware` reads sensors when available, performs calculations, communicates over MAVlink, or anything else in the code.
Note that most actions in the firmware are on timers, so not everything happens every time a `tick` service is called.
For example, the GPS sensor only creates information at 5-10 Hz, so it only gets read at that rate.

During this `tick` call, the `rosflight_firmware` communicates with the `rosflight_io` node using MAVlink.
In hardware, this communication happens over a serial connection, but we simulate this serial connection with a UDP connection when in sim.

After a `tick` completes, the `sil_board` publishes the resulting PWM commands over the `sim/pwm_output` topic to the `forces_and_moments` node.
The `forces_and_moments` node first unmixes the PWM commands and then computes the aerodynamic forces and moments acting on the airframe based on motor/prop characteristics and the aerodynamic coefficients of the aircraft.
Note that these calculations are only as accurate as the model in the `forces_and_moments` code.

The `forces_and_moments` node produces forces and moments, which it publishes over the `sim/forces_and_moments` topic to the `dynamics` node.
The `dynamics` node then adds other forces, like gravity and any collision forces, and integrates the state of the aircraft using an RK4 integration step.
The new truth state is published to whatever node is subscribed to the `sim/truth_state` topic.
Note that the `dynamics` node also creates and publishes wind truth to the `sim/wind_truth` topic.

The `visualizer` node refers to the visualizer used, i.e. RViz, Gazebo Classic, HoloOcean, etc.
The visualizer usually just subscribes to the true state and adjusts the visualization accordingly.

!!! note "A note on visualizers"

    Remember that different visualizers implement different modules.
    Gazebo Classic, for example, handles the dynamic integration while the `standalone_sim` (using RViz as the visualizer) just visualizes the model and the trajectory.

    The "visualizer" box in Fig 1 is therefore a placeholder, since the actual visualizer node might take up more than one module.

The `sensors` module receives the true state data and generates sensor data according to the true state.
This sensor data gets sent over various topics (i.e. `sim/standalone_sensors/XXX`) to the `sil_board`.

Finally, the `time_manager` node is in charge of regulating the simulation time, and publishes the `clock` topic to **all** nodes.
Note that if you don't want simulation time to be different than system time, you don't need the `time_manager` node.

## Module Descriptions

This section has more specific information on what each module does and its responsibilities in `rosflight_sim`.

### Time Manager
The `time_manager` node is in charge of regulating simulation time.
In ROS2, every node has a default parameter named `use_sim_time` (note that you don't have to declare this parameter--it comes by default).
By default, this parameter is set `false`.

When `use_sim_time == true`, however, the node will listen to the `clock` topic as its internal time source.
This means that all timers, calls to `get_clock()`, or any other time for that node will be based off of the `clock` topic.

### Sim Manager

### SIL Board

### Sensors

### Forces and Moments

### Dynamics

## Swapping out modules

This section describes how one would swap out modules, i.e. a different dynamics module.

## Node configuration for the different visualizers

This section describes how each visualizer supported by ROSflight uses the different modules described above.

### Standalone Sim

### Gazebo

### HoloOcean

## Adding your own visualizer

This section describes the process to add your own visualizer.
