# Parameter Management

## Overview

In ROSplane, all internal variables are represented as ROS2 parameters.
This enables easy loading, tuning, and saving parameters of various ROSplane modules without needing to rebuild.
ROS2 parameters can also be changed dynamically, enabling live editing and tuning of the system. 

For a good introduction to ROS2 parameters and the CLI tools, see the [ROS2 parameter CLI documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).
For an introduction to the parameter system using the ROS2 client libraries, see [ROS2 parameter client libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html) documentation.

## Parameter Manager class

A `param_manager` class has been created to manage the ROS interface required when dealing with parameters.
Specifically, a `param_manager` object handles the declaration and updates of parameters.
Since a base class publicly owns the `param_manager`, derived classes will have access to any parent class's parameters, if needed.
This also allows derived classes to define parameters close to where they will be used in the derived class, helping with readability.

For example, the `controller_base` class declares a `frequency` variable, which defines the rate of the control loops.
This same parameter is needed in other derived classes, like the `controller_successive_loop` class.
Since we declare the parameter in the `controller_base` class, the `controller_successive_loop` class will have access to the `frequency` parameter without having to redefine it.

## Usage

### Declaration

To use the `param_manager` to define your own parameters, do the following:

1. Declare an instance of `param_manager` in your class, or ensure that a parent class has a public or protected instance of `param_manager`.
2. Initialize the `param_manager` object with a pointer to the ROS2 node object associated with the parameters.
3. Use `param_manager::declare_param(std::string <PARAM_NAME>, <PARAM>)` to declare parameters of type double, bool, or string, where `<PARAM>` is the default value for the parameter.
    - Use `param_manager::declare_int(std::string <PARAM_NAME>, <PARAM>)` to declare an integer parameter.

These steps will register your parameters with ROS2, allowing you to change them dynamically or load them from a launch file.

### Using parameters in code

After declaring the parameters with `param_manager::declare_param` or `param_manager::declare_int`, you need to allocate variables in your code to hold the values of the parameters.
Get the parameter value by using the appropriate function call:

- `param_manager::get_double(std::string <PARAM_NAME>)`
- `param_manager::get_bool(std::string <PARAM_NAME>)`
- `param_manager::get_string(std::string <PARAM_NAME>)`
- `param_manager::get_int(std::string <PARAM_NAME>)`

Note that the return type of `param_manager::get_int` is `int_64`, since that is how ROS2 internally stores integers.

### Defining parameters with a parameter file

We recommend using a YAML file to hold all of the parameters for a given node.
This file can be loaded at launch time so that all parameters are updated with the values in the launch file.
This means you don't have to change the default values in code (which would require a rebuild) to make sure a node gets launched with the correct values.

To do this, add the `parameters=["/path/to/parameter/file"]` to a node's launch argument.
See `rosplane.launch.py` for an example.