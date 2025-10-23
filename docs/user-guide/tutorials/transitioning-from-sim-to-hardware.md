# Transitioning from Sim to Hardware

This section specifically addresses tools that help users simulate fixed-wing aircraft. 

ROSplane is a fixed-wing specific branch of the ROSflight software that allows users to cater autopilot code and functionality to fixed wing UAV's in a variety of configurations. Modular architecture and clearly defined interfaces provide users with the ability to adapt the tool to their specific needs. One particularly useful tool is the ROSplane simulation, which allows users to input an aerodynamic model of their aircraft and then fly in simulation with the equivalent autopilot code that would be used during a physical flight. 


## ROSplane Sim2Real
This section of the user manual addresses capabilities and recommondations for the ROSplane simulation for fixed-wing aircraft. 
ROSplane simulation allows users to insert aerodynamic models of their aircraft and test in simulation.
This is useful for controller tuning and validating new functionality in simulation before hardware tests.

The following pages provide a step-by-step user guide with information on how to use two open source aerodynamic modeling software tools (XFlyer5 and OpenVSP) to compute the aerodynamic control and stability derivatives required for the ROSplane aerodynamic model.

Following this guide will result in aerodynamic coefficients that are sufficiently close to the real aircraft aero coefficients.
For more information on the specific aerodynamic model used in ROSplane, see Chapter 4, Section 4.1 - 4.4 of "Small Unmanned Aircraft: Theory and Practice, by Randy Beard and Tim McLain."
[Google Drive link to file](https://drive.google.com/file/d/10iq7L_kAAdkjCFoq4EBRaT1u212BbyJ7/view)

Here is a link to a recently submitted conference paper outlining ROSplane capabilities and the sim-to-real workflow. 
[Link to file](https://arxiv.org/abs/2510.01041)

An example of the aircraft derivatives used in ROSplane simulation can be found in a parameter file located in the ROSflight workspace:
"rosflight_ros_pkgs/rosflight_sim/params/anaconda_dynamics.yaml". 

Users can create a custom dynamics file for their aircaft in the same "params" folder and then request it in the sim launch command by specifying the aircraft name. 

Example: User introduces a new aircraft dynamics file for a Cessna.
- New Dynamics File in "params" Folder: "cessna_dynamics.yaml"
- Command to Open Simulation of Cessna: "ros2 launch rosflight_sim fixedwing_sim_io_joy.launch.py aircraft:=cessna"


### Introduction and Disclaimer

This manual provides guidance on the basic features of XFLR5 and OpenVSP. It is not exhaustive and does not guarantee the accuracy of outputs. Validation through simulation and physical testing is recommended before flight.

The XFLR5 and OpenVSP sections discuss the step-by-step process for aircraft modeling and analysis using each tool. 
The Appendix section provides tips for working through common errors, links to helpful resources, and explanations of basic aerodynamics principles. 

### Table of Contents

* [XFLR5](user-manual-xflr5.md)
* [OpenVSP](user-manual-openvsp.md)
* [Appendix](user-manual-appendix.md)

You can also download this user manual as a single file and then open it in your browser to render it.
[Google Drive link to file](https://drive.google.com/file/d/10-37yCIjK796dnT-5MB5-q3M8o2nCi5_/view)


### Where to Start

Users should feel free to start with the tool they are most comfortable with. 
Xflr5 and OpenVSP produce similar models but each have unique strengths and application layout. 
Each tool's user manual should be done in order, but there is no requirement to use Xflr5 before OpenVSP. 

OpenVSP:
* Allows users to model the entire aircraft (fuselage, wing, landing gear, propellers, etc)
* Often provides a more accurate drag estimate
* Analyses take longer to run

Xflr5:
* Allows users more options when analyzing and visualizing aerodynamics and stability and control performance
* Allows airfoil optimization 
* Requires manual analysis of individual airfoils
* Can import/export AVL-compatible aircraft files

Appendix:
* Includes an aerodynamics crash course, common errors, notation and nomenclature outlines, and additional useful resources for further study 

When choosing which tool to start with, use whichever tool you are more comfortable with or use the tool that has capabilities you value. 
We recommend anyalizing your aircraft in both tools and comparing results to obtain the most accurate aircraft model. 




