# Controller Software Architecture

## Overview

The controller makes use of inheritance to make the controller more modular.
This creates a hierarchy of classes that each take on a responsibility.
This means that a user that modifies the controller only has to inherit from a particular class make the modifications only to the portions of the code that matter to them, and then continue to use the other parts of the controller.
An example of this is provided in the code.
The total energy controller exemplifies this by inheriting and only changing just a few key functions.
The aim of this architecture is to make the developer's job easier.

## Inheritance Scheme

The controller starts off with the `controller_ros` class.
This contains all of the interface with ROS.
The next layer, `controller_state_machine`, implements a basic state machine for the controller.
These states control which commands are happening at what time.
An example of how this may be used is, ensuring the aircraft is at altitude before attempting major maneuvers.
Following the state machine is the actual control scheme.
By default we use a successive loop closure scheme, `controller_successive_loop`.
This layer calculates the control errors and necessary actuator deflections to track commands.

This scheme was chosen to limit code duplication.
A composition structure would have resulted in code duplication between controllers.
Guides on how and where to modify each part of this structure can be found on the respective pages in the Developer Guide.
This page does go over the basics of how to implement a controller changes in terms of overall architecture.

## Implementing A New Controller 

The total energy controller in the `rosplane` package, shows in a practical way how to implement a new controller.
