# Customizing ROScopter

The ROScopter core functionality is intentionally lean, meaning the feature set is small.
While this increases understandability and can improve research productivity, it also means that the core ROScopter functionality will not satisfy all researcher's needs.
Additionally, research involves designing and testing *novel* features and algorithms, so no autopilot will have all the functionality desired by a researcher.

Thus, to be useful to researchers, ROScopter has been designed to be as modular and extensible as possible.
In other words, ROScopter has been **designed to be modified, customized, and extended**.
This enables researchers to do research with minimal effort.

This page describes how ROScopter is structured to allow researchers maximum flexibility.
We also give some examples of how ROScopter could be modified.

!!! warning "ROScopter architecture"
    This page will not go over the ROScopter architecture, but we do reference it.
    Please see the [ROScopter architecture documentation](./roscopter-architecture.md) for a description of each module before reading through this page.

## Intended workflow for ROScopter

## Modifying a single node

### Dependency on ROS 2
Each module in ROScopter (e.g. `path_planner`, `path_manager`, `controller`) is implemented as a ROS 2 node.
Each node has a clear interface with the rest of ROScopter defined by its ROS 2 interfaces (i.e. publishers, subscribers, service servers, etc.).
Each module communicates with the rest of ROScopter only over these ROS 2 interfaces.

This dependency on ROS 2 removes dependencies between the implementations of each module.
In other words, the regardless of how the `estimator` node is implemented (e.g. EKF vs invariant EKF vs any other estimator variant), as long as it has the same publishers and subscribers as the core ROScopter `estimator` node, then it will integrate seamlessly with the rest of ROScopter.

Thus, researchers can replace any node in ROScopter with their application code.
All that is required for the new code is that it has the same ROS 2 interfaces as the core ROScopter node.

!!! example "Replacing a core ROScopter node"
    A researcher studying model predictive control (MPC) wants to use ROScopter.
    The researcher has already implemented his controller as a separate C++ library, and it takes in reference trajectories and outputs low-level controller commands.

    Since the MPC controller takes in a trajectory and returns low-level control commands, the ideal node to replace would be the `trajectory_follower`.
    The researcher has already implemented it as a C++ library, so all he needs to do is write a ROS 2 wrapper around his library to have the same ROS 2 interfaces as the ROScopter `trajectory_follower`.

    Another option would be to create a derived class from the `trajectory_follower` interface class that forwards function calls to his library.

The dependency on ROS 2 also allows seamless cross-language support (for languages supported by ROS 2).
For example, any C++ node could be replaced by a Python node---as long as the ROS 2 interfaces are the same, ROScopter will compile and run.

!!! example "Replacing a core ROScopter node with a Python node"
    Another researcher is studying model predictive control (MPC) and wants to use ROScopter.
    The researcher has already implemented her controller in Python, and it takes in reference trajectories and outputs low-level controller commands.

    Since the MPC controller takes in a trajectory and returns low-level control commands, the ideal node to replace would be the `trajectory_follower`.
    The researcher has already implemented it in Python, so all she needs to do is write a ROS 2 wrapper around her library to have the same ROS 2 interfaces as the ROScopter `trajectory_follower`.

    The core ROScopter `trajectory_follower` is written in C++, so another option would be to rewrite the Python code in C++ and implement it as in the previous example.

Additionally, each module in ROScopter has a single responsibility, further removing dependencies between modules and functionality in ROScopter.

!!! example "Replacing a core ROScopter node - Example 3"
    A researcher interested in state estimation wants to use ROScopter.

    Since the researcher only wants to test new state estimation algorithms, the `controller`, `path_manager`, `path_planner`, or `trajectory_follower` modules do not need to be modified.
    This is because all relevant code is in the `estimator` node, whose single responsibility is to perform state estimation.

### Inheritance pattern
Each module in ROScopter has been designed with an inheritance pattern and is written in C++.
The base class is an abstract class and defines the ROS 2 interfaces for the node and declares *work* functions for the node.
These *work* functions are where the node does its "work" like estimating the state for the `estimtor` or creating a trajectory for the `path_manager`.
They are declared as pure virtual functions and thus must be implemented by a derived class.

This inheritance pattern reduces boilerplate code (i.e. rewriting the ROS 2 interfaces) for users.

## Modifying groups of nodes
