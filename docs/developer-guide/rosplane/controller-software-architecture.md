# Controller Software Architecture

## Overview

The controller makes use of inheritance to make the controller more modular.
This creates a hierarchy of classes that each take on a responsibility.
This means that a user that modifies the controller only has to inherit from a particular class make the modifications only to the portions of the code that matter to them, and then continue to use the other parts of the controller.
An example of this is provided in the code.
The total energy controller exemplifies this by inheriting and only changing just a few key functions.
The aim of this architecture is to make the developer's job easier.

## Inheritance Scheme

The controller starts off with the `controller_base` class.
This contains all of the interface with ROS.
The next layer, `controller_state_machine`, implements a basic state machine for the controller.
These states control which commands are happening at what time.
An example of how this may be used is, ensuring the aircraft is at altitude before attempting major maneuvers.
Following the state machine is the actual control scheme.
By default we use a successive loop closure scheme, `controller_successive_loop`.
This layer calculates the control errors and necessary actuator deflections to track commands.

<!-- TODO: add diagram for inheritance -->

## Implementing A New Controller 

<!-- TODO: add diagram for inheriting a new controller. -->

The total energy controller in the `rosplane` package, shows in a practical way how to implement a new controller.
This section is meant to only give a high level overview of how this can be done.
The first step is to identify where your changes should be made.
This means determining which class the change belongs in.
Consult the class pages for more information on where the best fit for your controller would be.
The next step is to define a new class that inherits from class you are changing.
Override the functions of interest.
Next if the inherited class is not at the bottom of the inheritance chain, you will have to modify (duplicate but only change the inheritance, this is to not break default behavior) the controller classes further down the chain to inherit from your class rather than the original.
This is to avoid an multiple inheritance problem (inheritance diamond).

<!-- TODO: add screenshots of where to make changes. -->

If this is done correctly, then you should be able to simply change between control schemes with only an argument to a launch file.
This will allow for easier testing where you can use a more tested controller initially and but swap to a new controller when convenient.

