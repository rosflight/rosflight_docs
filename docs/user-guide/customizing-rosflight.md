# Customizing ROSflight

The ROSflight core functionality is intentionally lean, meaning the feature set is small.
While this increases understandability and can improve research productivity, it also means that the core ROSflight functionality will not satisfy all researcher's needs.
Additionally, research involves designing and testing *novel* features and algorithms, so no autopilot will have all the functionality desired by a researcher.

Thus, to be useful to researchers, ROSflight has been designed to be as modular and extensible as possible.
In other words, ROSflight has been **designed to be modified, customized, and extended**.
This enables researchers to do research with minimal effort.

This page describes how ROSflight is structured to allow researchers maximum flexibility.
We also give some examples of how elements of ROSflight could be modified.

!!! tip "Customizing ROSflight firmware vs ROSflight autonomy stacks"

    As described in [the overview page](./overview.md), ROSflight is split into a companion computer and an embedded flight control unit (FCU).
    The companion computer has the ROSflight autonomy stacks while the FCU runs the ROSflight firmware and handles sensor and actuator I/O.

    Modules on the companion computer and modules on the FCU are designed differently, so the methods for customizing the ROSflight autonomy stacks and the ROSflight firmware are different.

!!! warning "ROSflight architecture"
    This page will not go over the [ROScopter](./roscopter/roscopter-overview.md)/[ROSplane](./concepts/rosplane-overview.md)/[ROSflight firmware](../developer-guide/firmware/code-architecture.md) architecture, but we do reference it.
    Please see the respective documentation for a description of each module before reading through this page.

## Customizing ROSflight firmware
The core [ROSflight firmware](./overview.md) resides on an embedded microcontroller.
Most users of ROSflight will not need to customize the ROSflight firmware, since the vast majority of the autonomy stack is on the companion computer.

If you need to customize the ROSflight firmware, first consider if you can accomplish what you need on the companion computer.
If you still need to customize the ROSflight firmware, make your changes and then follow the [building and flashing guide](../developer-guide/firmware/building-and-flashing.md) to flash it on your board.

However, since ROSflight currently supports only [limited hardware](./concepts/flight-controller-setup.md), users may need to write a new board-specific support package for the ROSflight firmware.
The ROSflight firmware depends on [an abstraction of a physical board](../developer-guide/firmware/code-architecture.md).
This means that supporting ROSflight on a new board does not require users to change the core ROSflight firmware code.
Instead, users only have to write drivers for sensors and implement the board-specific functions in the interface file.

## Customizing Autonomy Stacks
The core ROSflight autonomy stacks, ROSplane and ROScopter, reside entirely on the companion computer and are comprised of groups of core modules.
Customizing these autonomy stacks involves changing, removing, or replacing a single module or groups of modules.

### Modifying a single node

#### Dependency on ROS 2
Each module in ROScopter and ROSplane (e.g. `path_planner`, `path_manager`, `controller`) is implemented as a ROS 2 node.
Each node has a clear interface with the rest of ROSflight defined by its ROS 2 interfaces (i.e. publishers, subscribers, service servers, etc.).
Each module communicates with the rest of ROSflight only over these ROS 2 interfaces.

This dependency on ROS 2 removes dependencies between the implementations of each module.
For example, regardless of how the ROScopter `estimator` node is implemented (e.g. EKF vs invariant EKF vs any other estimator variant), as long as it has the same publishers and subscribers as the core ROScopter `estimator` node, then it will integrate seamlessly with the rest of ROScopter and ROSflight.

Researchers can replace any node in ROSflight with their application code.
All that is required for the new code is that it has the same ROS 2 interfaces as the core ROSflight node.

??? example "Replacing a core ROScopter node"

    A researcher studying model predictive control (MPC) wants to use ROSflight.
    The researcher has already implemented his controller as a separate C++ library, and it takes in reference trajectories and outputs low-level controller commands.

    Since the MPC controller takes in a trajectory and returns low-level control commands, the ideal node to replace would be the ROScopter `trajectory_follower`.
    The researcher has already implemented it as a C++ library, so all he needs to do is write a ROS 2 wrapper around his library to have the same ROS 2 interfaces as the ROSflight `trajectory_follower`.

    Another option would be to create a derived class from the ROScopter `trajectory_follower` interface class that forwards function calls to his library.

The dependency on ROS 2 also allows seamless cross-language support (for languages supported by ROS 2).
For example, any C++ node could be replaced by a Python node---as long as the ROS 2 interfaces are the same, ROSflight will compile and run.

??? example "Replacing a core ROScopter node with a Python node"

    Another researcher is studying model predictive control (MPC) and wants to use ROSflight.
    The researcher has already implemented her controller in Python, and it takes in reference trajectories and outputs low-level controller commands.

    Since the MPC controller takes in a trajectory and returns low-level control commands, the ideal node to replace would be the `trajectory_follower`.
    The researcher has already implemented it in Python, so all she needs to do is write a ROS 2 wrapper around her library to have the same ROS 2 interfaces as the ROSflight `trajectory_follower`.

    The core ROSflight `trajectory_follower` is written in C++, so another option would be to rewrite the Python code in C++ and implement it as in the previous example.

Additionally, each module in ROScopter and ROSplane has a single responsibility, further removing dependencies between modules and functionality in ROSflight.

??? example "Replacing a core ROSplane node - Single responsibility principle"

    A researcher interested in state estimation wants to use ROSflight.

    Since the researcher only wants to test new state estimation algorithms, the ROSplane `controller`, `path_manager`, `path_planner`, or `path_follower` modules do not need to be modified.
    This is because all relevant code is in the `estimator` node, whose single responsibility is to perform state estimation.

#### Inheritance pattern
Most of the modules in ROSflight have been designed with an inheritance pattern and are written in C++.
The base class is an abstract class and defines the ROS 2 interfaces for the node and declares *work* functions for the node.
These *work* functions are where the node does its job, like estimating the state for the `estimator` or creating a trajectory for the `path_manager`.
They are declared as pure virtual functions and thus must be implemented by a derived class.

This inheritance pattern reduces boilerplate code (i.e. rewriting the ROS 2 interfaces) for users.

Implementing custom code in ROSflight can be done by subclassing/inheriting from the interface class and implementing the custom functionality.

### Modifying groups of nodes
In some cases, application code can take the place of several core ROSflight nodes.
The procedure for replacing groups of nodes in ROSflight is the same as it is to replace a single node.
As long as the ROS 2 interfaces remain the same between the other core ROSflight nodes, the new node will interface seamlessly with the rest of ROSflight.

The linear, cascaded flow of information through ROSflight makes this easy.
Modules typically only depend on output from the module just above, so changes to upstream modules don't affect how downstream modules are implemented.

??? example "Replacing a group of nodes"

    A researcher studying spline-based path planning wants to use ROSplane.
    The new spline-based planning approach takes in information about the enviroment (i.e. obstacles, start and end locations, etc.) and outputs course, airspeed, and altitude commands.

    This new code replaces the functionality provided by the core ROSplane navigation stack (the `path_planner`, `path_manager`, and `path_follower`).
    The researcher can therefore replace all these nodes with the new spline-based controller.
    The new node just needs to output commands to the ROSplane `controller` as the ROSplane `path_follower` does.

### Intended workflow for the core ROSflight autonomy stacks
When using ROScopter or ROSplane, users should first determine which core ROSflight nodes will be replaced by the new application code.
Depending on the application, a single node or a group of nodes may need to be modified as described above.
Some applications may not need to modify any core ROSflight nodes.

After determining which core nodes to modify/replace, the application code should be tested in ROSflight sim.
Note that the simulation makes assumptions about the vehicle's mass, size, inertia matrix, etc.
For ROScopter, these values are based off a real quadcopter (the Holybro x650 frame), so they are somewhat realistic.
ROSplane's aerodynamic coefficients are based off of the [aerodynamic analysis](./tutorials/transitioning-from-sim-to-hardware.md) of a RMRC Anaconda.
If you will not do hardware experiments, these parameters will likely work just fine.

If hardware experiments will be performed, then users should set airframe-specific parameters appropriately in ROSflight sim and also in each of the ROSflight nodes (many ROSflight nodes need to know about the mass of the vehicle).
This helps ease the transition from sim to real tests.

Remember that ROScopter and ROSplane run entirely on the companion computer, so no software changes are necessary when transitioning from simulation to hardware.
