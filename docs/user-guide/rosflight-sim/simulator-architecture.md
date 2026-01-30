# Simulator Architecture

This document describes the architecture of the simulator code.
It describes each node's function and role in the simulator, as well as how different visualizers require different configurations of nodes.

It also details what you would need to do to [customize the sim to your needs](#customizing-the-simulator).

!!! note "Prerequisites"

    This guide assumes that you have installed and set up the ROSflightSim, as detailed in [the tutorials](../tutorials/setting-up-rosflight-sim.md).
    

## Big Picture - Sim Architecture

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
| :---: |
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

A single simulation loop starts with the `sil_board` node.
The `sil_board` by default executes a simulation loop when a ROS2 timer fires.
This corresponds to a single iteration of the main loop in the ROSflight firmware (see the [relevant source code](https://github.com/rosflight/rosflight_firmware/blob/main/src/rosflight.cpp), the `run()` function).

On a tick, the `rosflight_firmware` code reads sensors when available, performs calculations, communicates over MAVlink, or anything else in the code.
Note that most actions in the firmware are on timers, so not everything happens every time `sil_board` ticks.
For example, the GPS sensor only creates information at 5-10 Hz, so it only gets read at that rate, not every time `sil_board` ticks.

During this tick, the `rosflight_firmware` also communicates with the `rosflight_io` node using MAVlink.
In hardware, this communication happens over a serial connection, but we simulate this serial connection with a UDP connection when in sim.

After a tick completes, the `sil_board` publishes the resulting PWM commands over the `sim/pwm_output` topic to the `forces_and_moments` node.
The `forces_and_moments` node first unmixes the PWM commands and then computes the aerodynamic forces and moments acting on the airframe based on motor/prop characteristics and the aerodynamic coefficients of the aircraft.
Note that these calculations are only as accurate as the model in the `forces_and_moments` code.

The `forces_and_moments` node produces forces and moments, which it publishes over the `sim/forces_and_moments` topic to the `dynamics` node.
The `dynamics` node then adds other forces, like gravity and any collision forces, and integrates the state of the aircraft using an RK4 integration step.
The new truth state is published to whatever node is subscribed to the `sim/truth_state` topic.
Note that the `dynamics` node also creates and publishes wind truth to the `sim/truth_wind` topic.

The `visualizer` node refers to the visualizer used, i.e. RViz, Gazebo Classic, HoloOcean, etc.
The visualizer usually just subscribes to the true state and adjusts the visualization accordingly.

!!! note "A note on visualizers"

    Remember that different visualizers implement different modules.
    Gazebo Classic, for example, handles the dynamic integration while the `standalone_sim` (using RViz as the visualizer) just visualizes the model and the trajectory.

    The "visualizer" box in Fig 1 is therefore a placeholder, since the actual visualizer node might take up more than one module.

The `sensors` module receives the true state data and generates sensor data according to the true state.
This sensor data gets sent over various topics (i.e. `sim/standalone_sensors/XXX`) to the `sil_board`.
RC commands are communicated to the `sil_board` similarly, where the RC commands are generated by the `rc` node.

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

ROSflight sim can be run with or without the `time_manager`.
If you are using the `time_manager`, note that you can toggle pause/play of the simulation using the `/time_manager/toggle_pause` service call.

!!! warning "When to use the `time_manager`"

    The `time_manager` is really only useful when you care about running faster or slower than real time or pausing and starting the simulation.
    If you don't care about this, don't run the `time_manager` node, and don't set the `use_sim_time` parameter of other nodes to `true`.
    This will free up resources otherwise used by the `time_manager`.

The `time_manager`'s main job is to publish the current time to the `clock` topic.
It needs to publish fast enough so that other timers on other nodes aren't stalled because the `clock` topic isn't coming fast enough.


The `time_manager` has some built in functionality to enable faster or slower than real time simulations.
This is done by configuring the parameters associated with the `time_manager` node.

| <span style="display: inline-block; width:150px">Parameter name</span> | Default value | Description |
| --- | --- | --- |
| `default_pub_rate_us` | 100.0 | Default interval the time manager will publish to the `clock` topic (in microseconds) |
| `real_time_multiplier` | 1.0 | Multiplier for configuring faster or slower than real time simulations |

!!! example "Pub rate example"

    Let's say I know that the fastest timer in my simulation environment runs at 400 Hz.
    Thus, that timer has to tick every 2.5 ms.
    The `default_pub_rate` parameter therefore needs to be **smaller than 2500** so that the timer runs reliably.

    Note that we have not tested all of the timing intricacies of the `clock` topic--set it to publish faster than you need and you probably won't run into issues.

!!! example "Real time multiplier example"

    If I want to run a simulation 2 times faster than real time, I would set `real_time_multiplier = 2.0`.
    If I wanted to run it 0.5 times as fast as real time, I would set it to 0.5.

### SIL Board
The `sil_board` is the simulated version of the physical flight controller.
It is composed of two parts: the **software-in-the-loop (SIL) board**, and **a ROS2 wrapper** that manages the interfaces with the rest of the simulation.

A more accurate image of the `sil_board` node than what is in Figure 1 can be seen below in Figure 2.
This figure is discussed in detail in the following subsections.

| ![SIL board architecture](../images/simulator_architecture_sil_board.svg) |
| :---: |
| Fig 2: More accurate description of the SIL board architecture with its two major parts: the ROS2 wrapper and the SIL board. Note that this is pseudocode--the "variable" and function names do not necessarily correspond to the actual variable names in the code.  |


!!! note "Naming"

    In implementation, the ROS2 executable corresponding to the "ROS2 wrapper" half of the overall SIL Board module is called `sil_board`.
    In this guide, we will refer to the ROS2 wrapper as `sil_board_ros`, and the actual board implementation as SIL board.

#### SIL Board

!!! warning "Important"

    "Board" here refers to a module that inherits from ["board.h"](https://github.com/rosflight/rosflight_firmware/blob/main/include/interface/board.h), the interface file defining all the functions a physical board must implement in order to run the `rosflight_firmware`.

The SIL board's responsibility is to perform the same tasks as the physical board.
It does this by inheriting from ["board.h"](https://github.com/rosflight/rosflight_firmware/blob/main/include/interface/board.h), thus implementing **all of the same functionality** as the board implementations that run on physical hardware.

In the implementation of the [ROSflight firmware](https://github.com/rosflight/rosflight_firmware), any `firmware` object created is passed a reference to a `board` object (which is an object that inherits from "board.h").
This happens the same way in hardware and in simulation.
The firmware uses this reference to the `board` object at the appropriate times to do things like get the current clock time, read sensor information, write motor commands, and so on.
In Fig 2, this is shown by the arrows flowing from the "ROSflight firmware" box to the "SIL Board" box.

Thus, the SIL board implements functions like `imu_read`, `gnss_read`, `rc_read`, `pwm_write`, etc.
Since we are in sim, instead of reading from the physical IMU when `imu_read()` is called, the `sil_board` loads the IMU data from the information received via subscription to the `sensors` node, which is responsible for creating the simulated sensor information.
Similarly, instead of writing PWM signals to the physical pins, the SIL board publishes those commands to the `sim/pwm_output` topic, which the `forces_and_moments` node uses to compute the aerodynamic forces and moments.
These publisher/subscriber interfaces are denoted in Fig 2.

#### ROS2 Wrapper
As shown in Fig 2, the ROS2 wrapper contains:

- An instantiation of the [**ROSflight firmware**](../overview.md#firmware), which is the same code that runs on the physical flight controller.
- The **SIL board object** (discussed previously)
- The **communication link** module (not shown in Fig 2)

Its main responsibility is to manage when the ROSflight firmware's `run()` function gets called.

!!! note "The `run()` function" 

    Remember that this `run()` function corresponds to a single execution loop of the firmware.
    In hardware, this `run()` function runs *very* fast (~350kHz on some hardware), though not everything in the firmware runs at that same rate since most functionality is hooked to hardware interrupts.
    It is unnecessary to run it this fast in sim, so we typically run it at the IMU rate, ~400Hz.

In the `sil_board_ros`, the `run()` function can get called in two ways:

1. From a ROS2 timer callback, or
2. From a ROS2 service served up by the `sil_board_ros` node.

Only one of these methods should be used at a time when running the firmware.
The timer is used in the standard configuration, as it models what happens in the real hardware.
The service server is useful when taking one step of the firmware at a time in order to analyze the effects step by step.

The timer frequency can be adjusted using the ROS2 parameter system.

### Sensors
The `sensors` module is responsible for generating simulated sensor measurements based on the current true state.
These sensor measurements are published to the `/sim/sensors/XXX` topics, where `XXX` is each sensor, i.e. `imu/data` or `baro`, etc.

The `sensors` node adds noise, walk, and biases to each sensor measurement to simulate the real-world data.
See the code for more information on how the noise is added.

Note that the `sensors` node subscribes to the `sim/forces_and_moments` topic (for the IMU measurement), the truth states, and the `status` topic.
The `status` topic is used to approximate when the motors are spinning so high-frequency noise can be added to the gyros.

Also note that the `sensors` node could have been implemented as a separate node for each sensor.
If you are adding a new sensor (e.g. camera), you could either change the `sensor` source code or create an entirely separate ROS2 node for your sensor.

### RC node
The `rc` module is responsible for publishing RC commands to the `sil_board`.
It takes the place of the physical RC receiver in hardware that typically communicates with the flight controller over SBUS or PPM.

As described in the [ROSflight tutorials](../tutorials/manually-flying-rosflight-sim.md#rc-transmitter-control) and [hardware concept pages](../hardware-and-rosflight/hardware-setup.md#joystick) pages, the `rc` node supports using a physical joystick or a simulated joystick like VimFly.
See the linked documents for more information.

If VimFly is not specified and a physical transmitter is not connected when the simulation is launched, it will default to **no direct RC control**.
This means that the arming and RC override functionality (usually performed by switches on the transmitter) need to be done using the below ROS2 service calls:
```bash
# Toggle arm
ros2 service call /toggle_arm std_srvs/srv/Trigger

# Toggle RC override
ros2 service call /toggle_override std_srvs/srv/Trigger
```

!!! note

    These service calls are only available when neither VimFly nor a physical transmitter are used.

Regardless of whether or not a transmitter is connected, the `rc` node publishes RC data to the `/rc` topic.
Each channel of this data is a value between 1000-2000, corresponding to the pulse width of the PWM signal (in microseconds).
As described above, this data is subscribed to by the `sil_board` directly.
**RC data does not flow through `rosflight_io`.**

### Forces and Moments
The `forces_and_moments` node is responsible for computing the **aerodynamic** forces and moments based on a model of the aircraft.
Other forces like gravity and collision forces are not included in the `forces_and_moments` node.

The `forces_and_moments` node takes in raw PWM commands published by the `sil_board` node over the `sim/pwm_output` topic, computes the forces and moments, and publishes those values over the `sim/forces_and_moments` topic.
These PWM commands correspond to what would be either servo deflections or motor throttle values on the physical aircraft.
To compute the forces and moments generated by those actuator commands, we need to convert the PWM commands into inputs used by our model.

#### Fixedwing
For fixedwing aircraft, our aerodynamic model is the model proposed in *Small Unmanned Aircraft: Theory and Practice* by Beard and McLain.
It takes in 4 commands, \(\delta_a, \delta_e, \delta_r, \in [-1,1]\) and \(\delta_t \in [0,1]\), corresponding to aileron, elevator, rudder servo commands, and throttle setting, respectively.
Thus, we need to convert the relevant PWM commands on the corresponding channels into these four servo and throttle setpoints.

For the "standard" airframe this is trivial, since the standard airframe maps a single PWM command to one of the 4 inputs to our model.
We just need to know what PWM channels correspond to which servo, and then convert the PWM command to within the correct range (i.e. [-1,1] or [0,1]).

For a non-standard airframe (i.e. vtail, like the [RMRC Anaconda](https://www.readymaderc.com/products/details/rmrc-anaconda-kit?srsltid=AfmBOopBO1pTJlXnkzJTptNt_7ki6yl3ING49Oe518JvIjyqUAUdg9OX)), the information sent by the firmware over the `pwm_output` topic does not correspond to the "standard" inputs required by our model.
This means we first have to unmix the actual PWM commands to get the equivalent "standard" commands.

The `forces_and_moments` node accomplishes this by querying the `sil_board` node through `rosflight_io` to determine the current values of the mixer.
It saves the mixer and unmixes the input PWM commands back to the "standard" commands.
We then can use our aerodynamic model to compute the forces and torques.

This process is shown in Fig 3.

| ![Flow of information through the forces and moments node](../images/simulator_architecture_fandm.svg) |
| :---: |
| Fig 3: Flow of information through the `forces_and_moments` node. The \(\delta_{r1}\) and \(\delta_{r2}\) values in the data of the `/sim/pwm_output` section refer to the right and left ruddervator commands used for a vtail aircraft. Note how the mixer is used in two places. |

!!! note

    We could skip all the mixing and unmixing and subscribe directly to the incoming "standard" commands before they go into the `sil_board` node.
    This, however, **reduces the realism** of the simulator, since it neglects any changes that could have been made by the firmware to those commands.

!!! warning "Max servo deflection"

    Make sure the `max_aileron_deflection_angle`, `max_elevator_deflection_angle`, and `max_rudder_deflection_angle` parameters are set correctly.
    This scales the PWM command from [-1, 1] to the actual physical angle used by the aerodynamic model.

    **If these are incorrect, the simulated aircraft will behave very differently than the physical one.**

#### Multirotors

The model used for the multirotor is simpler than the fixedwing aerodynamic model.
It consists of a model of the motor and propeller as well as some drag parameters.

Since each PWM command from the firmware maps directly to a single motor, we don't have to unmix the commands to compute the forces and moments.
Instead, we use the position and direction of the rotors to directly compute the forces and moments using the motor/prop equations.
More information on these equations can be found in chapters 4 and 14 of *Small Unmanned Aircraft: Theory and Practice*.

However, we do need to accurately set the positions of the motors.
This is done through the `rotor_dists`, `rotor_radial_angles`, and `rotor_rotation_directions` parameters of the `forces_and_moments` node.

### Dynamics
The `dynamics` node is responsible for maintaining the true state of the vehicle and for adding environmental effects.
It can be thought of as the "world node", since it is the node that implements physical phenomena like gravity, collisions, state integration, etc.

!!! note

    At the time of writing, collisions have not been added to the `dynamics` node.
    If you are interested in fixing this, please see [the GitHub issue](https://github.com/rosflight/rosflight_ros_pkgs/issues/214).

The `dynamics` node subscribes to the `sim/forces_and_moments` topic and publishes to the `sim/truth_state` and `sim/truth_wind` topics.
The `sim/truth_state` topic contains the 19-DoF state for a generic rigid body (3 for position, 4 for quaternion orientation, 6 for angular and linear velocities, 6 for angular and linear accelerations).
The `sim/truth_wind` topic contains a 3-vector for each component of the wind (in the inertial frame).

!!! tip "Wind in sim"
    Currently, no wind is generated by default, but a wind model like the one in chapter 4.4 of *Small Unmanned Aircraft: Theory and Practice* by Beard and McLain could be used.

    TODO: Add details about the wind models in sim!

#### Setting the simulation state
It can be useful to instantiate the simulation at a particular point in state space, to run a particular experiment, to avoid takeoff, and so on.

The `dynamics` node has a service server that allows users to set the simulation state (the 19-DoF state) to whatever value you want, called the `dynamics/set_sim_state` service.
Note that if an estimator is running, it will likely do something crazy if you set the sim state while it is running.

## Customizing the simulator
Because of the modular nature of ROS2, nodes can be swapped out with minimal effort.
As long as the inputs and outputs (the ROS2 interfaces, i.e. publishers, subscribers, services) remain the same, the new module should fit in seamlessly with the rest of the simulator.

#### Important implementation details
Each module described above (except for the `sil_board`) has been implemented as a C++ node with an interface class and a single derived class.
For example, the `sensors` module has an interface class, `SensorInterface`, which the implemented class, `StandaloneSensors` inherits from.

The interface class defines all of the ROS2 interfaces and the key functions that a derived class must implement for the code to function correctly.

!!! note "Why is it done this way?"

    Designing the architecture in this way defines a "contract" in the interface class.
    If that contract is satisfied (which is enforced by the compiler), then the derived class code will work with the rest of `rosflight_sim` (assuming, of course, that the code in the derived class is correct).

    This makes it easier and quicker to create different implementations of the same interface class.
    For example, the `forces_and_moments` node has a different implementation of the aerodynamic model for the fixedwing and for the multirotor.
    Instead of duplicating all of the code for the ROS2 interfaces, we move it to the interface class.
    Additionally, we require that derived classes implement some key functions like `update_forces_and_torques`.
    Thus, for the fixedwing and multirotor forces and moments, all we do is inherit from the interface class and implement the required functions, and we're good to go!

The following table lists each module and the corresponding interface class, as well as the functions required by the interface class.
These functions are the functionality that you would be required to implement if you were to swap out a module for a different one.

| Module | <span style="display: inline-block; width:200px">Interface</span> | Required functions |
| --- | --- | --- |
| Time Manager | `TimeManagerInterface` | `update_time`, `get_seconds`, `get_nanoseconds` |
| SIL Board | None | None |
| Sensors | `SensorInterface` | `imu_update`, `imu_temperature_update`, `mag_update`, `baro_update`, `gnss_update`, `sonar_update`, `diff_pressure_update`, `battery_update` |
| Forces and Moments | `ForcesAndMomentsInterface` | `update_forces_and_torques`, `get_firmware_parameters` |
| Dynamics | `DynamicsInterface` | `apply_forces_and_torques`, `compute_truth`, `compute_wind_truth` |

!!! note "Programming languages"

    The majority of the simulation code is written in C++.
    If you want to use a different language when replacing a module, you will have to implement the interface class in that language (i.e. Python).

    In most cases, you should be able to go line by line and replace syntax.
    Or you could have an LLM do it for you.

#### Example customizations

??? example "Example use case: Different aerodynamic model"

    The aerodynamic model in the forces and moments node is not a high fidelity model, but makes some assumptions in order to simplify the resulting equations.
    Let's say I wanted to implement a different aerodynamic model in order to increase the fidelity of my simulator.

    All I would need to do would be to create a new class that inherits from the forces and moments interface class (`ForcesAndMomentsInterface`), which handles the ROS2 interfaces and defines the functions that my forces and moments node needs to have.
    All I do is then

    1. implement those functions with my custom aerodynamic model,
    1. add it to the `CMakeLists.txt` (so that it is built),
    1. add it to the launch file instead of the default `forces_and_moments` node, and I'm done!

??? example "Example use case: JSBsim"

    [JSBsim](https://jsbsim.sourceforge.net/index.html) is an open-source, widely used flight dynamics software.
    For example, both [Ardupilot](https://ardupilot.org/dev/docs/sitl-with-jsbsim.html) and [PX4](https://docs.px4.io/v1.15/en/sim_jsbsim/index.html) both support JSBsim in their simulation envirnoments.
    This is an example of how JSBsim could be integrated into ROSflight for better aerodynamics.

    JSBsim would replace both the `forces_and_moments` and the `dynamics` nodes.
    The first step would be to combine the interface classes for both modules into a single ROS2 wrapper.
    The next step would be to incorporate the JSBsim API into that ROS2 wrapper.
    Remember that as long as the inputs (`sim/forces_and_moments` topic) and outputs (`sim/truth_state`) are correct, it will work with the rest of ROSflight!

## Node configuration for the different visualizers
This section describes how each visualizer natively supported by ROSflight uses the different modules described above.

### Standalone Sim
The "standalone sim" is a lightweight version of the simulator that uses ROS2 RViz as the visualization engine (i.e. to visualize the 3D flight path of the vehicle).

| ![Standalone sim software modules](../images/simulator_architecture_standalone_sim.svg) |
| :--: |
| Fig 4: Standalone sim software modules |

The standalone simulator uses every module described above and optionally uses the `standalone_time_manager` to manage the simulation time.

### Gazebo Classic
Gazebo Classic is a (now EOL'd) robotic simulator.
It was previously supported by ROSflight, so it is supported now.
However, it currently does not work on versions of above ROS2 Humble, and should be replaced with Gazebo (the currently maintained version of Gazebo).
In this guide, we often refer to "Gazebo Classic" as Gazebo.

| ![Gazebo sim software modules](../images/simulator_architecture_gazebo_sim.svg) |
| :--: |
| Fig 5: Standalone sim software modules |

Gazebo uses plugins to interface with Gazebo world attributes.
Furthermore, Gazebo handles dynamic integration internally, meaning we don't have to do it.
Thus, the dynamics node is written as a plugin to Gazebo, and only manages publishing the true state and the true wind state.

Also note that Gazebo publishes the `/clock` topic, so the `standalone_time_manager` should **not** be used with Gazebo.
Additionally, the Gazebo node only publishes this topic at ~10Hz, which is too slow for most applications (especially the sensors).
Thus, **we recommend not setting the [`use_sim_time`](#time-manager) parameter to true when using Gazebo**, or you will get timing errors.

### HoloOcean
[HoloOcean](https://robots.et.byu.edu/holoocean/) is a photorealistic simulator built off of Unreal Engine 5.

| ![HoloOcean sim software modules](../images/simulator_architecture_holoocean_sim.svg) | 
| :--: |
| Fig 6: Standalone sim software modules |

The structure of HoloOcean is very similar to the standalone simulator, and can be used identically.
HoloOcean can do much more (e.g. dynamic integration or other sensors), and would require the user to modify relevant ROSflight sim nodes.
Camera nodes could be added by creating a separate ROS2 publisher with the camera data (instead of adding the camera to the sensor module).

Currently, we use the HoloOcean API to "teleport" the model between the true states computed by the dynamics node.

## Adding your own visualizer

Though ROSflight supports three different simulators out of the box, we anticipate that users may need to use ROSflight with their own simulator.
The process for using ROSflight with your own simulator follows the same principles as the [customization section above](#customizing-the-simulator).

The first step is to determine what modules in the standalone simulator you can leverage without modification.
Then, determine any modules that need to be merged with your visualizer (e.g. the dynamics node or others).

When merging the ROSflight modules into your code, make sure to maintain the same ROS2 interfaces (publishers, subscribers, services, etc.).
If you do, it is likely the rest of the ROSflight simulation will "just work" with your implementation.

Happy simulating! ✈️
