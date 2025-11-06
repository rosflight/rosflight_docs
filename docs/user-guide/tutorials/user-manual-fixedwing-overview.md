# Transitioning from Sim to Hardware

This section specifically addresses tools that help users simulate fixed-wing aircraft. 

ROSplane is a fixed-wing specific branch of the ROSflight software that allows users to cater autopilot code and functionality to fixed wing UAVs in a variety of configurations. Modular architecture and clearly defined interfaces provide users with the ability to adapt the tool to their specific needs. One particularly useful tool is the ROSplane simulation, which allows users to input an aerodynamic model of their aircraft and then fly in simulation with the equivalent autopilot code that would be used during a physical flight. 


## ROSplane Sim2Real
This section of the user manual addresses capabilities and recommendations for the ROSplane simulation for fixed-wing aircraft. 
ROSplane simulation allows users to insert aerodynamic models of their aircraft and test in simulation.
This is useful for controller tuning and validating new functionality in simulation before hardware tests.

The following pages provide a step-by-step user guide with information on how to use two open source aerodynamic modeling software tools (XFlr5 and OpenVSP) to compute the aerodynamic control and stability derivatives required for the ROSplane aerodynamic model.

Following this guide will result in aerodynamic coefficients that are sufficiently close to the real aircraft aero coefficients.
For more information on the specific aerodynamic model used in ROSplane, see Chapter 4, Section 4.1 - 4.4 of [*Small Unmanned Aircraft: Theory and Practice*](https://drive.google.com/file/d/10iq7L_kAAdkjCFoq4EBRaT1u212BbyJ7/view) by Randy Beard and Tim McLain.

Here is a link to a recently submitted [conference paper](https://arxiv.org/abs/2510.01041) outlining ROSplane capabilities and the sim-to-real workflow. 



### Introduction and Disclaimer

This manual provides guidance on the basic features of XFLR5 and OpenVSP. It is not exhaustive and does not guarantee the accuracy of outputs. Validation through simulation and physical testing is recommended before flight.

!!! warning
    Xflr5 and OpenVSP are trademarks of their respective organizations. This webiste is not affiliated with or endorsed by them. This only provides helpful tips on how rosplane users can use these free open-source software tools to model and analyze their aircraft for integration with the ROSplane simulation.

The XFLR5 and OpenVSP sections discuss the step-by-step process for aircraft modeling and analysis using each tool. 
The Appendix section provides tips for working through common errors, links to helpful resources, and explanations of basic aerodynamics principles. 


You can also download this [user manual](https://drive.google.com/file/d/10-37yCIjK796dnT-5MB5-q3M8o2nCi5_/view) as a single file and then open it in your browser to render it.

### Where to Start

The workflow for integrating a new aircraft model into the ROSflight simulation is:

1. Obtain prerequisite dimension and inertia measurements
2. Model your aircraft
3. Perform aerodynamic and stability analyses 
4. Integrate into ROSflight 
5. Test in simulation!

#### Modeling Your Aircraft and Running Aerodynamic Analyses

!!! note "Which tool do I use?"
    You should feel free to start with the tool you are most comfortable with. 
    Xflr5 and OpenVSP produce similar models but each have unique strengths and application layout. 
    Each tool's user manual should be done in order, but there is no requirement to use Xflr5 before OpenVSP. 

[Xflr5](./user-manual-xflr5.md):

* Allows users more options when analyzing and visualizing aerodynamics and stability and control performance

* Allows airfoil optimization 

* Requires manual analysis of individual airfoils

* Can import/export AVL - compatible aircraft files

[OpenVSP](./user-manual-openvsp.md):

* Allows users to model the entire aircraft (fuselage, wing, landing gear, propellers, etc)

* Often provides a more accurate drag estimate

* Analyses take longer to run

When choosing which tool to start with, use whichever tool you are more comfortable with or use the tool that has capabilities you value. 
We recommend analyzing your aircraft in both tools and comparing results to obtain the most accurate aircraft model. 

#### ROSflight Integration 

After modeling your aircraft and running the aerodynamic/stability/control analyses, follow [this guide](./user-manual-fixedwing-integration.md) to add your model to the ROSflight simulation. 

This guide contains details on the specific files and parameters you need to edit to fly your aircraft in simulation.

#### Additional Resources and Further Study

The [Appendix](./user-manual-appendix.md) contains an aerodynamics crash course, common errors, notation and nomenclature outlines, and additional useful resources/links for further study. 

!!! tip
    If you have questions during your modeling and analysis process, make sure to look here for helpful information and tips. 




