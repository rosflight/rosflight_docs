# Tuning Performance in Sim

This tutorial guides you through systematically tuning control gains for optimal flight performance in simulation.
You'll learn to use PlotJuggler, RQT dynamic reconfigure, and the signal generator to tune both firmware and high-level control loops.

!!! note
    We will be tuning the firmware and ROScopter as an example as we work through this tutorial.
    Tuning ROSplane is an identical process, just with different gains and different control loops.

This tutorial will walk you through:

- Setting up tuning tools (PlotJuggler, RQT, Signal Generator)
- Understanding the control hierarchy and what to tune
- Systematic PID tuning methodology
- Tuning firmware controller gains (rate and attitude control)
- Tuning ROScopter gains
- Analyzing performance with data visualization

!!! note
    While we do cover some PID tuning methodology, this guide is mainly meant to 

    - **Introduce the tools** that we use when flying with ROSflight, and
    - Help you get a feel for an **autonomy stack architecture**

    See the [improving firmware performance](../concepts/improving-firmware-performance.md) guide for more information on tuning the firmware controller and estimator.

## Prerequisites

- [Setting up ROScopter in Sim](./setting-up-roscopter-in-sim.md)
- Basic understanding of PID control theory
- Some familiarity with ROScopter system architecture

## Overview

### Controller Hierarchy

ROScopter implements a hierarchical control structure with multiple nested control loops.
These control loops ultimately feed into the firmware's control loops.

This image shows a flow diagram for how information moves through the ROScopter controller to the firmware controller and then to the physical aircraft.

![ROScopter and the firmware control loop interactions](../images/roscopter_and_firmware_controllers.png)

!!! note
    This diagram only shows the `autopilot` and `sil_board` nodes.
    It does not include the `trajectory_follower` node.
    We're focusing on the `autopilot` and `sil_board` controllers for now, but we will end up needing to tune all of the nodes to get the best performance possible.

    For more information on these nodes, see the [previous](./setting-up-roscopter-in-sim.md) guides.


Each box or "command type" in the "ROScopter Controller" and "Firmware" sections is a level where a user can insert and publish commands.
The commands then flow down through the chain from wherever the user inserted.

!!! example "Example: ROScopter controller information flow"
    For example, if I want to send north, east, and down (NED) velocity and yaw rate commands to ROScopter, I would publish commands at the `NED-Vel YawRate` level, as described in the `roscopter_msgs/msg/ControllerCommands` message type.

    The autopilot would then convert those to NED acceleration/Yaw rate commands, which then are sent to the roll, pitch, yawrate, throttle controller, and then to the firmware angle controller.
    The commands are then finally sent through the mixer to the motors.

### What We're Tuning

Each of the command types in the above diagram are implemented as PID control loops (with the exception of the NED-Accel command type).
Thus, to tune the controller, we'll need to tune each PID gain.

This turns out to be a lot of gains!
To keep it all straight, we are going to start by tuning the lowest levels first and gradually working our way up.
We won't do all of the loops, but we will do enough to show you how we typically do it.

We will also introduce several tools that help us keep track of all the gains.

## Setting Up Tuning Environment

### Launch Simulation and ROScopter

First, set up the complete simulation environment:

```bash
# Terminal 1: Launch multirotor simulation
ros2 launch rosflight_sim multirotor_standalone.launch.py use_vimfly:=true

# Terminal 2: Initialize firmware (if needed)
ros2 launch rosflight_sim multirotor_init_firmware.launch.py

# Terminal 3: Launch ROScopter autonomy stack
ros2 launch roscopter_sim sim.launch.py
```
### Install and Launch PlotJuggler

PlotJuggler provides real-time data visualization for analyzing control performance.
We'll use it to plot data live as we send control commands to the vehicle.

<video autoplay loop muted playsinline>
  <source src="../../images/plotjuggler.webm" type="video/webm">
  Video of PlotJuggler in sim.
</video>

Launching PlotJuggler:
```bash
# Install PlotJuggler if not already installed
sudo apt install ros-humble-plotjuggler-ros

# Launch PlotJuggler
ros2 run plotjuggler plotjuggler
```

**PlotJuggler Setup:**

1. Click "Start" → Select "ROS2 Topic Subscriber"
2. Subscribe to key topics:
   - `/imu/data` - IMU measurements
   - `/estimated_state` - State estimation
   - `/controller_commands` - High-level commands
   - `/command` - Low-level motor commands
1. Show them on the plot by clicking and dragging the data streams to the plot window.

Plotjuggler is a powerful tool.
We won't cover all the functionality here.
We'll leave it to you to plot the topics you need as we go through this tutorial.

### Launch RQT Dynamic Reconfigure

RQT is a plugin-based GUI tool distributed with ROS2 (the desktop version).
The dynamic reconfigure plugin offered by RQT allows real-time parameter adjustment.

![Example of RQT Dynamic Reconfigure plugin](../images/rqt_example.png)

Each of the values shown in the image above are parameters (ROS2 parameters) of the `autopilot` node.
ROS2 parameters make it easy to change those parameters on the fly.
See [the official documentation on parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) for more information.

We have implemented all of the controller gains as ROS2 parameters.
This makes it easy to use RQT Dynamic Reconfigure to adjust and tune the controller.

Launch with:
```bash
# Launch RQT with dynamic reconfigure plugin
rqt --standalone rqt_reconfigure
```

### Launch Signal Generator

The signal generator provides systematic test inputs like square waves, step inputs, etc. to an output of choice.

```bash
ros2 run roscopter_tuning signal_generator
```

TODO: Continue here!

## Understanding Control Architecture

### Data Flow

```
Waypoints → Position Controller → Velocity Controller → Attitude Commands → Firmware
                (ROScopter)         (ROScopter)         (ROScopter)       (ROSflight)
```

### Key Topics for Tuning

Monitor these topics during tuning:

- **`/estimated_state`**: Current vehicle state (position, velocity, attitude)
- **`/high_level_command`**: Commands from trajectory follower to autopilot
- **`/controller_commands`**: Commands from autopilot to firmware
- **`/command`**: Final motor commands from firmware
- **`/imu/data`**: Raw sensor measurements

## Firmware Controller Tuning

Firmware controllers run at high frequency (typically 500-1000 Hz) and provide basic stabilization.

### Rate Controller Tuning

The rate controller is the innermost loop and should be tuned first.

#### Step 1: Set Conservative Initial Gains

Start with conservative gains in the firmware parameter file:

```bash
# Edit firmware parameters
nano ~/rosflight_ws/src/rosflight_ros_pkgs/rosflight_sim/params/multirotor_firmware.yaml
```

Look for these parameters:
```yaml
# Rate controller gains
PID_ROLL_RATE_P: 0.05
PID_ROLL_RATE_I: 0.0
PID_ROLL_RATE_D: 0.001

PID_PITCH_RATE_P: 0.05
PID_PITCH_RATE_I: 0.0
PID_PITCH_RATE_D: 0.001

PID_YAW_RATE_P: 0.05
PID_YAW_RATE_I: 0.0
PID_YAW_RATE_D: 0.001
```

#### Step 2: Generate Test Signals

Use the signal generator to create step responses:

```bash
# Generate step response for roll rate
ros2 service call /start_step_signal std_srvs/srv/Trigger

# Generate sine wave for frequency response
ros2 service call /start_sine_signal std_srvs/srv/Trigger
```

#### Step 3: Analyze Response in PlotJuggler

In PlotJuggler, plot:
- **Commanded vs Actual**: `/command/roll` vs `/imu/data/angular_velocity.x`
- **Error**: Difference between commanded and actual rates
- **Control Output**: Motor commands from firmware

#### Step 4: Tune Using Standard PID Methods

**Proportional Gain (P):**
1. Increase P gain until system responds quickly to commands
2. Too high: Oscillations and instability
3. Too low: Sluggish response

**Derivative Gain (D):**
1. Add D gain to reduce overshoot and oscillations
2. Start with P/10 as initial D value
3. Increase until overshoot is minimized

**Integral Gain (I):**
1. Add I gain last to eliminate steady-state error
2. Start very small (P/100)
3. Increase slowly to avoid oscillations

### Attitude Controller Tuning

After rate controller is stable, tune the attitude controller.

#### Attitude Controller Parameters

```yaml
# Attitude controller gains
PID_ROLL_ANGLE_P: 2.0
PID_ROLL_ANGLE_I: 0.0
PID_ROLL_ANGLE_D: 0.1

PID_PITCH_ANGLE_P: 2.0
PID_PITCH_ANGLE_I: 0.0
PID_PITCH_ANGLE_D: 0.1

PID_YAW_ANGLE_P: 1.0
PID_YAW_ANGLE_I: 0.0
PID_YAW_ANGLE_D: 0.05
```

#### Attitude Tuning Process

1. **Command step inputs** using VimFly or service calls
2. **Monitor response** in PlotJuggler:
   - Commanded vs actual attitude
   - Settling time and overshoot
   - Steady-state error

### Using RQT for Real-Time Tuning

For parameters exposed to ROS2, use RQT dynamic reconfigure:

1. Open RQT reconfigure
2. Navigate to `/rosflight_io` node
3. Adjust parameters in real-time
4. Observe immediate changes in PlotJuggler

## ROScopter High-Level Controller Tuning

High-level controllers run at lower frequency (typically 100-200 Hz) and provide autonomous flight capabilities.

### Position Controller Tuning

The position controller converts position errors into velocity commands.

#### Position Controller Parameters

Edit ROScopter parameters:

```bash
# Edit ROScopter controller gains
nano ~/rosflight_ws/src/roscopter/roscopter/params/multirotor.yaml
```

Key parameters:
```yaml
autopilot:
  ros__parameters:
    # Position control gains
    PID_P_n: 2.0    # North position gain
    PID_I_n: 0.0    # North integral gain
    PID_D_n: 0.5    # North derivative gain
    
    PID_P_e: 2.0    # East position gain
    PID_I_e: 0.0    # East integral gain
    PID_D_e: 0.5    # East derivative gain
    
    PID_P_d: 3.0    # Down position gain
    PID_I_d: 0.0    # Down integral gain
    PID_D_d: 1.0    # Down derivative gain
```

#### Position Tuning Process

1. **Load waypoint mission** with step changes in position
2. **Monitor topics** in PlotJuggler:
   - `/estimated_state/position`
   - `/high_level_command/position`
   - Position error over time

3. **Tune systematically**:
   - Start with P gain only
   - Increase until good tracking with minimal overshoot
   - Add D gain to reduce oscillations
   - Add I gain if steady-state error exists

### Velocity Controller Tuning

The velocity controller converts velocity commands into attitude commands.

#### Velocity Controller Parameters

```yaml
autopilot:
  ros__parameters:
    # Velocity control gains
    PID_P_u: 1.5    # Body-frame X velocity gain
    PID_I_u: 0.0    # Body-frame X integral gain
    PID_D_u: 0.2    # Body-frame X derivative gain
    
    PID_P_v: 1.5    # Body-frame Y velocity gain
    PID_I_v: 0.0    # Body-frame Y integral gain
    PID_D_v: 0.2    # Body-frame Y derivative gain
    
    PID_P_w: 2.0    # Body-frame Z velocity gain
    PID_I_w: 0.1    # Body-frame Z integral gain
    PID_D_w: 0.3    # Body-frame Z derivative gain
```

#### Velocity Tuning Process

1. **Command velocity steps** using service calls or waypoint missions
2. **Analyze response** in PlotJuggler:
   - Commanded vs actual velocity
   - Attitude commands generated
   - Settling time and accuracy

## Systematic Tuning Methodology

### Step-by-Step Process

1. **Start with Conservative Gains**
   - Set all gains low initially
   - Ensure system is stable before proceeding

2. **Tune Inner Loops First**
   - Rate controller → Attitude controller → Velocity → Position
   - Each loop must be stable before tuning outer loops

3. **Use Consistent Test Inputs**
   - Step responses for transient analysis
   - Sine waves for frequency response
   - Square waves for disturbance rejection

4. **Monitor Key Metrics**
   - **Rise Time**: How quickly system responds
   - **Overshoot**: Maximum deviation from setpoint
   - **Settling Time**: Time to reach steady state
   - **Steady-State Error**: Final error after settling

### PID Tuning Guidelines

#### Ziegler-Nichols Method

1. **Find Critical Gain**: Increase P until system oscillates
2. **Measure Period**: Time of oscillation at critical gain
3. **Calculate Gains**:
   - P = 0.6 × Critical Gain
   - I = 2P / Period
   - D = P × Period / 8

#### Manual Tuning Rules

- **Too much P**: Oscillations, instability
- **Too little P**: Sluggish response, large steady-state error
- **Too much I**: Slow oscillations, overshoot
- **Too little I**: Steady-state error remains
- **Too much D**: High-frequency noise amplification
- **Too little D**: Overshoot, slow settling

## Using Signal Generator

The signal generator provides systematic test inputs for controller analysis.

### Available Signal Types

```bash
# Step response
ros2 service call /start_step_signal std_srvs/srv/Trigger

# Sine wave
ros2 service call /start_sine_signal std_srvs/srv/Trigger

# Square wave
ros2 service call /start_square_signal std_srvs/srv/Trigger

# Triangle wave
ros2 service call /start_triangle_signal std_srvs/srv/Trigger

# Sawtooth wave
ros2 service call /start_sawtooth_signal std_srvs/srv/Trigger
```

### Signal Parameters

Configure signal characteristics:

```bash
# Set signal amplitude
ros2 param set /signal_generator amplitude 0.5

# Set signal frequency
ros2 param set /signal_generator frequency 0.1

# Set signal offset
ros2 param set /signal_generator offset 0.0
```

### Practical Signal Usage

- **Step Response**: Analyze transient performance, overshoot, settling time
- **Sine Wave**: Frequency response analysis, phase margin
- **Square Wave**: Disturbance rejection, robustness testing
- **Triangle Wave**: Slow ramp tracking performance
- **Sawtooth Wave**: Asymmetric response characteristics

## Practical Tuning Examples

### Example 1: Aggressive vs Conservative Tuning

**Conservative Gains** (stable but slow):
```yaml
PID_P_n: 1.0
PID_D_n: 0.2
```

**Aggressive Gains** (fast but may oscillate):
```yaml
PID_P_n: 3.0
PID_D_n: 0.8
```

### Example 2: Altitude Control Tuning

Common issue: Poor altitude tracking during aggressive maneuvers.

**Solution**:
1. Increase `PID_P_d` (down position gain)
2. Add `PID_I_d` for steady-state accuracy
3. Tune `PID_P_w` (vertical velocity gain)

### Example 3: Oscillation Problems

**Symptoms**: Vehicle oscillates in hover or during tracking.

**Diagnosis**:
- High-frequency oscillation → Reduce D gains
- Low-frequency oscillation → Reduce P gains or increase D gains
- Slow oscillation → Reduce I gains

## Advanced Analysis

### Frequency Domain Analysis

Use PlotJuggler's FFT capabilities:

1. Collect data during sine wave inputs
2. Analyze frequency response
3. Determine gain and phase margins
4. Optimize for desired bandwidth

### Multi-Input Testing

Test multiple axes simultaneously:

```bash
# Enable multi-axis signal generation
ros2 param set /signal_generator multi_axis true
```

### Performance Metrics

Calculate quantitative metrics:
- **Integral Absolute Error (IAE)**
- **Integral Square Error (ISE)**
- **Maximum Overshoot Percentage**
- **Settling Time (2% criterion)**

## Saving Tuned Parameters

### Firmware Parameters

After tuning firmware parameters:

```bash
# Save parameters to firmware memory
ros2 service call /param_write std_srvs/srv/Trigger
```

### ROScopter Parameters

After tuning ROScopter parameters, save to YAML file:

```bash
# Edit parameter file with final tuned values
nano ~/rosflight_ws/src/roscopter/roscopter/params/multirotor.yaml
```

## ROSplane Tuning

Tuning ROSplane follows a similar process with these differences:

- **Fixed-wing specific controllers**: Airspeed, altitude, course control
- **Different dynamics**: Longitudinal and lateral-directional modes
- **Additional parameters**: Bank angle limits, airspeed constraints

The same tools (PlotJuggler, RQT, Signal Generator) and methodology apply to ROSplane controller tuning.

## Troubleshooting Common Issues

### Vehicle Won't Arm After Parameter Changes

**Solution**: Check parameter bounds and ensure all required parameters are set.

```bash
# Check parameter status
ros2 service call /param_get rosflight_msgs/srv/ParamGet "{name: 'PID_ROLL_RATE_P'}"
```

### Parameters Not Taking Effect

**Solution**: Restart the system or reload parameters.

```bash
# Reload firmware parameters
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename: 'multirotor_firmware.yaml'}"
```

### Unexpected Oscillations

**Solution**: Return to conservative gains and retune systematically.

```bash
# Reset to default parameters
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
```

## Review

You have successfully learned to tune ROScopter control performance. You should now be able to:

- **Set up tuning environment**: Launch PlotJuggler, RQT, and Signal Generator
- **Understand control hierarchy**: Recognize firmware vs high-level control loops
- **Tune systematically**: Follow inner-to-outer loop tuning methodology
- **Use analysis tools**: Interpret PlotJuggler data and adjust parameters accordingly
- **Apply PID theory**: Implement standard tuning techniques for optimal performance

## Next Steps

1. **Practice with different scenarios**: Try aggressive maneuvers, payload changes, wind disturbances
2. **Advanced techniques**: Explore adaptive control, feedforward compensation
3. **Hardware deployment**: Apply learned tuning skills to real hardware
4. **Custom controllers**: Develop and tune your own control algorithms

### Additional Resources

- [Control Theory Background](../concepts/control-theory.md): Detailed control system theory
- [Parameter Reference](../concepts/parameter-configuration.md): Complete parameter documentation
- [Hardware Tuning Guide](../concepts/hardware-tuning.md): Considerations for real hardware
