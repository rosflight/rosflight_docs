# Transitioning from Sim to Hardware

ROSflight provides users the ability to fly their aircraft and test software in simulation. 
It allows users to cater autopilot code and functionality to fixed wing UAVs in a variety of configurations. 
Modular architecture and clearly defined interfaces provide users with the ability to adapt the tool to their specific needs. 
The **ROSplane** and **ROScopter** simulations tools allow users to input an aerodynamic model of their aircraft and then fly in simulation with the equivalent autopilot code that would be used during a physical flight. 
This is useful for controller tuning and validating new functionality in simulation before hardware tests.

The following pages provide a step-by-step user guide with information on how to identify and integrate aerodynamic models for ROSflight simulations.


!!! warning "Disclaimer"
    This manual is not exhaustive and does not guarantee the accuracy of outputs. Validation through simulation and physical testing is recommended before flight.


### Where to Start

The workflow for integrating a new aircraft model into the ROSflight simulation is:

1. Obtain prerequisite dimension and inertia measurements
2. Model your aircraft
3. Perform aerodynamic and stability analyses 
4. Integrate into ROSflight 
5. Test in simulation!

Start here for instructions on [Fixedwing](./user-manual-fixedwing-overview.md) system identification, modeling, and simulation.

Start here for instructions on [Quadrotor](./user-manual-quadrotor-overview.md)system identification, modeling, and simulation.





