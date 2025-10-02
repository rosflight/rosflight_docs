# Transitioning from Sim to Hardware

## ROSplane Sim2Real
ROSflight simulation allows users to insert aerodynamic models of their aircraft and test in simulation.
This is useful for controller tuning and validating new functionality in sim before hardware tests.

See the attached user guide for information on how to use XFlyer5 and OpenVSP to compute the aerodynamic control and stability derivatives required for the ROSflight aerodynamic model.
These derivatives are found in a parameter file.






# XFLR5 & OpenVSP User Manual


System Identification for Novel Aircraft Integration into Rosflight Ecosystem

Dr. Tim McLain and Joseph Ritchie

## Introduction and Disclaimer


This manual provides guidance on the basic features of XFLR5 and OpenVSP. It is not exhaustive and does not guarantee the accuracy of outputs. Validation through simulation and physical testing is recommended before flight.




## Table of Contents


* [XFLR5](#xflr5)
* [OpenVSP](#openvsp)
* [Appendix](#appendix)





![Xflr5 Logo](../../../assets/user_manual_assets/pictures/Xflr5_Logo.svg)
## XFLR5




**Quick Navigation:**
* [Navigation](#navigation)
	+ [Direct Foil Design](#direct-foil-design)
	+ [Xfoil Inverse Design](#xfoil-inverse-design)
	+ [Xfoil Direct Analysis](#xfoil-direct-analysis)
	+ [Wing and Plane Design](#wing-and-plane-design)
* [Creating and Analyzing Airfoils](#creating-and-analyzing-airfoils)
	+ [Direct Foil Design](#direct-foil-design)
	+ [Adding NACA Foils](#adding-naca-foils)
	+ [Adding .dat Files](#adding-dat-files)
	+ [Adding Flaps](#adding-flaps)
	+ [Inverse Foil Design](#inverse-foil-design)
	+ [XFoil Direct Analysis](#xfoil-direct-analysis)
* [Creating a Plane](#creating-a-plane)
	+ [Information Checklist](#information-checklist)
	+ [Making the Plane](#making-the-plane)
	+ [Mass and Inertia Inputs](#mass-and-inertia-inputs)
* [Running an Analysis](#running-an-analysis)
	+ [Basic Analysis](#basic-analysis)
	+ [Stability Analysis](#stability-analysis)
* [Reading Graphs](#reading-graphs)
	+ [Overview](#overview)
	+ [OpPoint Viewer](#oppoint-viewer)
	+ [Polar Viewer](#polar-viewer)
	+ [Root Locus Viewer](#root-locus-viewer)
	+ [Time Response Viewer](#time-response-viewer)
	+ [Pressure Viewer](#pressure-viewer)
	+ [Changing Graphs](#changing-graphs)
	+ [Customizing Visuals](#customizing-visuals)
* [Common Errors](#common-errors)
	+ [Outside Flight Envelope](#outside-flight-envelope)
	+ [Can't be Interpolated](#cant-be-interpolated)
	+ [Negative Lift](#negative-lift)
* [References List and Helpful Links](#references-list-and-helpful-links)




### Navigation


To download XFLR5, go to the [XFLR5 website](https://sourceforge.net/projects/xflr5/files/latest/download).


Once downloaded, open the XFLR5.exe file to start the program.


![Navigation 1](../../../assets/user_manual_assets/xflr5%20pictures/Navigation%201%20download%20xflr.svg)
Once XFLR5 is open, you will see the main menu screen.


The menu bar is located at the top of the screen.


Under the File tab you can open and save projects. In the Module tab you can access the different modules of XFLR5.


![Navigation 2](../../../assets/user_manual_assets/xflr5%20pictures/Navigation%202.svg)
These modules contain the different tools and features of XFLR5.


![Modules 1](../../../assets/user_manual_assets/xflr5%20pictures/modules%201%20pic%20and%20text.svg)
#### Direct Foil Design


Direct Foil Design is where you will create, import, edit, and save each airfoil for your aircraft.
![Modules 2](../../../assets/user_manual_assets/xflr5%20pictures/modules%202%20direct%20foil%20design.svg)
#### Xfoil Inverse Design


Xfoil Inverse Design is where you reverse engineer an optimal foil shape by editing airfoil performance curves.  

![Modules 1](../../../assets/user_manual_assets/xflr5%20pictures/modules%203%20foil%20inverse%20design%201.svg)
#### Xfoil Direct Analysis


Xfoil Direct Analysis is where you can analyze airfoils in each expected flight condition (determined by a range of Reynolds numbers).  

![Modules 2](../../../assets/user_manual_assets/xflr5%20pictures/modules%204%20direct%20analysis%201.svg)
#### Wing and Plane Design


Wing and Plane Design is where you can use your previously analyzed airfoils to model and analyze the performance/stability of a complete aircraft.  

![Modules 2](../../../assets/user_manual_assets/xflr5%20pictures/modules%205%20wing%20and%20plane%20design%201.svg)












### Creating and Analyzing Airfoils


#### Direct Foil Design


The first step in modeling an aircraft is defining the airfoils used in the aircraft.  

 > Module > Direct Foil Design  

![Direct foil design 1](../../../assets/user_manual_assets/xflr5%20pictures/Direct%20foil%20design%201.svg)
Prerequisite: You must already know the NACA code or have a .dat file on hand for every airfoil on each section of the wing, elevator, and rudder.  

Direct Foil Design is where you will create, import, edit, and save each airfoil for your aircraft.  

![Direct foil design 2](../../../assets/user_manual_assets/xflr5%20pictures/Direct%20foil%20design%202.svg)
 SAVE CONSTANTLY!


#### Adding NACA Foils


NACA foils are standardized airfoils commonly defined by a 4-digit code (ex. 2412).  

 To import a NACA foil: > Foil > Naca Foils  

![Naca 1](../../../assets/user_manual_assets/xflr5%20pictures/naca1.svg)
 Enter the NACA code and the number of desired panels for the airfoil you want to add.   

 Click OK.  

![Naca 2](../../../assets/user_manual_assets/xflr5%20pictures/naca2.svg)
 Enter the name of your airfoil into the popup box and click OK.  

![Naca 3](../../../assets/user_manual_assets/xflr5%20pictures/naca3.svg)



#### Adding .dat Files


You can also add a custom airfoil by importing a .dat file of the airfoil shape.  

 To add a .dat airfoil: > File > Open > Select .dat file from your computer > Open  

![dat1](../../../assets/user_manual_assets/xflr5%20pictures/dat1.svg)
 The newly imported airfoil will appear here.


#### Adding Flaps


Once your airfoil is added to your project, you can edit the airfoil and add control surfaces.  

  

 To add a flap to the airfoil: right click on your airfoil, then click "Set Flap"  

![flaps 1](../../../assets/user_manual_assets/xflr5%20pictures/flaps1.svg)
 Click the box for a leading edge (LE) and/or tailing edge (TE) flap.  

  

 Input the flap characteristics (angle, position along chord, position of hinge)  

  

 Click OK  

![flaps 2](../../../assets/user_manual_assets/xflr5%20pictures/flaps2.svg)
 Enter the name of your new airfoil and click OK. Click "Overwrite" if you don't want to add a new airfoil.  

![flaps 3](../../../assets/user_manual_assets/xflr5%20pictures/flaps3.svg)
 You can edit any airfoil by right clicking it and using the drop-down menu.  

  

 Repeat these steps for every airfoil in your aircraft with control surfaces.  

![flaps 4](../../../assets/user_manual_assets/xflr5%20pictures/flaps4.svg)
#### Inverse Foil Design


Inverse Foil Design allows you to create a custom airfoil based off desired performance characteristics.  

 > Module > Xfoil Inverse Design  

![inverse foil design section 1](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%201.svg)
 See the graphic below for an overview of the tools in Xfoil Inverse Design  

![inverse foil design section 2](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%202.svg)
 To start, import the foil you want to edit.  

 Click on the "extract foil" button.  

 Click on the foil you want to modify then click "OK".  

![inverse foil design section 3](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%203.svg)
 Next, choose what flight condition context you want to modify your airfoil in and activate the spline function.  

![inverse foil design section 4](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%204.svg)
 Move the spline to create the curve you want your airfoil to reflect.  

![inverse foil design section 5](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%205.svg)
 After your custom spline is complete, click "execute" to create your spline.  

![inverse foil design section 6](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%206.svg)
 Once the airfoil is created, it can be exported by selecting the "Foil" tab at the top and then the "Store Foil" option.  

![inverse foil design section 7](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%207.svg)
 Name your airfoil, then click the "OK" button to save your new airfoil.
 ![inverse foil design section 8](../../../assets/user_manual_assets/xflr5%20pictures/inverse%20foil%20design%20section%208.svg)
 This airfoil can now be added to your project and analyzed in Xfoil Direct Analysis.


#### XFoil Direct Analysis


Each airfoil must be individually analyzed at every expected flight condition before it can be used in a full aircraft analysis.  

 > Module > XFoil Direct Analysis  

  

 This tool basically analyzes how the airfoils will behave at a variety of airspeeds and angles of attack.  

  

 Prerequisites: Know the expected Reynolds numbers of each flight condition (airspeed etc.) for the aircraft.  

![direct analysis section 1](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%201.svg)
 After opening XFoil Direct Analysis, you will see the following viewer.  

![direct analysis section 1](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%202.svg)
 Click "Analysis", then click "Batch Analysis".  

![direct analysis section 2](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%203.svg)
 Note: Batch analysis performs many analyses simultaneously across a variety of flight conditions to save time. If you want to analyze the airfoil at a single Reynolds number, choose "Define an Analysis" instead.  

![direct analysis section 3](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%203.svg)
 Select the airfoils you want to analyze in the top list, specify the range of Reynolds numbers over which you want to analyze the airfoil, select any other desired polar specifications (alpha sweep etc), and click "Analyze". Click "Close" when finished.  

![direct analysis section 4](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%204.svg)
 The graphs on the viewer will update after the analysis is completed. In the top menu, the chart icon on the right will show you polar views (drag, lift polars etc.)  

![direct analysis section 5](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%205.svg)
 The OpPoint viewer icon is on the left. This viewer will show you performance distributions across the airfoil at specific flight situations (e.g. pressure distributions at a specific angle of attack, flow separation, etc.)  

![direct analysis section 6](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%206.svg)
 To change any of the graphs, right click on the graph, then select "Current Graph" and "Define Graph Settings". This will allow you to customize the axes and any other graph information.  

![direct analysis section 7](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%207.svg)
 To change the lines on the graphs (color, weight, etc) right click on the airfoil or operating point on the list on the left then select your desired line characteristics.  

![direct analysis section 8](../../../assets/user_manual_assets/xflr5%20pictures/direct%20analysis%20section%208.svg)









### Creating a Plane


#### Information Checklist


![creating a plane 1](../../../assets/user_manual_assets/xflr5%20pictures/creating%20a%20plane%201.svg)
#### Making the Plane


Open the "Wing and Plane Design" tab in the top left menu. 
 To begin modeling your aircraft, click on "Plane" on the top left menu. Then select "Define a New Plane".  

![wing and plane design 1](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%201.svg)
 Once the "Plane Editor" opens, you will be able to define the wing, elevator, rudder, and weight distribution of the aircraft.  

![wing and plane design 2](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%202.svg)
 To create a wing, first click the "Symmetric" button (depending on your wing design). Next, begin inputting your wing data into the table below. Input the airfoil and airfoil position for each section of the wing into the table. Make sure your measurements are in meters. (Reminder: your airfoils must already be created in Foil Design and analyzed in Xfoil Direct Analysis).  

![wing and plane design 3](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%203.svg)
 Note: You should have a measurement for every major change in the wing. If there is a change in airfoil (such as the start of end of a flap), you will have two overlapping measurements at that transition point.  

![wing and plane design 4](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%204.svg)
 Once the wing is defined, make sure that your wing matches your design, and that each airfoil transition overlaps correctly. (Note: the control surfaces do not need to be angled unless you are specifically analyzing at an actuated position. A zero angle will still suffice for a stability analysis as long as the control surface hinge exists).  

![wing and plane design 5](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%205.svg)
 Next, check the paneling on the wing. On the lower right menu, toggle the "Surfaces" button off and the "Panels" button on. Set the Y-panels over long sections to fill the space and set the distribution to "Cosine" so the edges of the sections have more panels than the middle. Set overlapped sections to 1.  

![wing and plane design 6](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%206.svg)
 Click "Save" when you are done.  

 Once your wing is defined and saved, define the position of the leading edge and the initial geometric tilt angle. 
 ![wing and plane design 7](../../../assets/user_manual_assets/xflr5%20pictures/wing%20and%20plane%20design%207.svg)
 Follow the same steps for the elevator and rudder.  

#### Mass and Inertia Inputs


After all your lifting surfaces are correctly defined, click on the "Plane Inertia" button to define the weight distribution of your aircraft.  

![mass and inertia inputs 1](../../../assets/user_manual_assets/xflr5%20pictures/mass%20and%20inertia%20inputs%201.svg)
 Input the mass and position of each important mass of the aircraft.  


![mass and inertia inputs 2](../../../assets/user_manual_assets/xflr5%20pictures/mass%20and%20inertia%20inputs%202.svg)

  

 Validate that your weight distribution is accurate by comparing your CG and inertia tensor values with experimental values.  

![mass and inertia inputs 3](../../../assets/user_manual_assets/xflr5%20pictures/mass%20and%20inertia%20inputs%203.svg)
 You can also validate your mass distribution by toggling on the "masses" button on the Plane Editor page and visually inspecting the location of the CG.






### Running an Analysis


#### Basic Analysis


A basic analysis will help you obtain information about lift, drag, moments from the airfoils, and other basic aerodynamic characteristics of your aircraft.  

 > Analysis > Define an Analysis  

![basic analysis 1](../../../assets/user_manual_assets/xflr5%20pictures/xflr%20basic%20analysis%201.svg)
 Once the Analysis Definition box opens, in the Polar Type tab choose your desired polar type. (Fixed lift is recommended because it automatically trims the aircraft)  

![basic analysis 2](../../../assets/user_manual_assets/xflr5%20pictures/xflr%20basic%20analysis%202.svg)
 In the Analysis tab, choose your desired analysis method (VLM2 is recommended) and whether or not to include viscous flows.  

![basic analysis 3](../../../assets/user_manual_assets/xflr5%20pictures/xflr%20basic%20analysis%203.svg)
 In the Inertia tab, check the "Use plane inertia" box and change nothing (assuming you already accurately defined your plane inertia). Otherwise, uncheck the box and define your Mass and CG location manually.  

![basic analysis 4](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%204.svg)
 In the Ref. dimensions tab, select the dimensions you want to use for your aerodynamic coefficient calculations.  

![basic analysis 5](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%205.svg)
 In the Aero data tab, ensure the density and viscosity measurements are accurate to your expected flight environment.  

  

 Leave "Ground Effect" unchecked, unless you are simulating takeoff and landing.  

![basic analysis 6](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%206.svg)
 In the Extra drag tab, add any extra drag areas and drag coefficients in the corresponding table. Xflr5 only accurately simulates airflow over the lifting surfaces of the aircraft, so the drag from the fuselage, landing gear, or any other parts of the aircraft need to be accounted for separately.  

  

 A simple way to estimate extra drag is to calculate the cross-sectional area of the other parts of the aircraft and use a drag coefficient from a similar shape (sphere, cone, etc).  

  

 Click "Save" when you are done.  

![basic analysis 7](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%207.svg)
 A popup tab will appear. Type your desired identifier or name of your analysis.  

 Click "OK" when you are done.  

![basic analysis 8](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%208.svg)
  

 Your analysis is now defined, and ou can now run it.  

 Choose your analysis from the left, define the parameters on the right, and click "analyze".  

![basic analysis 9](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%209.svg)
 Wait until the analysis is completed, the click the close button.  

![basic analysis 10](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%2010.svg)
 On the home screen, you will see the analysis has updated. On the left you can select specific points of the analysis. In the middle you will see charts visualizing the analysis data. You can move between visualization by using the plot menu at the top.  

![basic analysis 11](../../../assets/user_manual_assets/xflr5%20pictures/basic%20analysis%2011.svg)
#### Stability Analysis


A stability analysis will allow you to evaluate the stability and control derivatives as well the time response of the aircraft to disturbances.  

  

**Stability Analysis Prerequisites:**  

* Airfoil polars analyzed (Xfoil Direct Analysis)
* Basic aerodynamic analysis performed
* Cm – alpha curve has a positive x-intercept
* Cm – CL curve must have a positive CL value at 0 Cm
* Aircraft inertia tensors and center of gravity are correct
* Control surface dimensions, position, and paneling must be accurate and appropriate


 You should actuate only one group of control surfaces in a stability analysis. For this reason, you will most likely need to run several stability analyses.  

 Example: have one stability analysis with ailerons activated to get the aileron control derivatives, and another analysis to get the elevator control derivatives.  

  

 To setup your stability analysis, select the aircraft you want to analyze.  

 > Analysis > Define a Stability Analysis  

![stability analysis 1](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%201.svg)
 Once the Stability Polar Definition box opens, define any relevant analysis constraints in the Analysis tab.  

![stability analysis 2](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%202.svg)
 In the Ref. dimensions tab, select the dimensions you want to use for the stability and control coefficient calculations.  

![stability analysis 3](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%203.svg)
 In the Mass and inertia tab, check the "Use plane inertia" box at the top and change nothing (assuming you already accurately defined your plane inertia); Otherwise, uncheck the box and define your Mass and CG location manually.  

![stability analysis 4](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%204.svg)
 In the "Control parameters" tab, select the control surfaces to analyze.  

  
To calculate control derivatives for each set of control surfaces (ailerons, rudder, elevator) it is recommended to perform a stability analysis for each set individually.  

  
It is not recommended to activate multiple control surfaces at once. Ailerons should be actuated in opposite directions and elevators are in the same direction.  

![stability analysis 5](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%205.svg)
 In the Aero data tab, ensure the density and viscosity measurements are accurate to your expected flight environment.  

![stability analysis 6](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%206.svg)
 In the Extra drag tab, add any extra drag areas and drag coefficients in the corresponding table. Xflr5 only accurately simulates airflow over the lifting surfaces of the aircraft, so the drag from the fuselage, landing gear, or any other parts of the aircraft need to be accounted for separately.  

  

 A simple way to estimate extra drag is to calculate the cross-sectional area of the other parts of the aircraft and use a drag coefficient from a similar shape (sphere, cone, etc).  

 Click "Save" when you are done.  

![stability analysis 7](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%207.svg)
 Select the analysis you want to run and double check your analysis is defined correctly. 
 Set the analysis settings, and click "Analyze".  

![stability analysis 8](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%208.svg)
 While the analysis is being performed you will see a "3D Panel Analysis" box displaying the test code. If there are no errors, the box will automatically close. In the case of errors, the box will remain open and you will need to close it manually.  

![stability analysis 9](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%209.svg)
 Feel free to look through the test code after the analysis is performed.  

 After the analysis is complete, save your work immediately.  

 To explore the analysis results, click on the analysis from the list on the left.  

![stability analysis 10](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%2010.svg)
 To see specific outputs or results, click on an operating point from the analysis.  

 Aerodynamic, stability, and control derivatives are in the box at the bottom left.  

![stability analysis 11](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%2011.svg)
 A more in-depth record of the analysis outputs can be found in the log file.  

 > Analysis > View Log File  

![stability analysis 12](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%2012.svg)
 The log file contains the complete outputs at each point of the analysis such as each stability and control derivative, state matrices, and stability response modes.  

![stability analysis 13](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%2013.svg)
 The analysis output tab on the bottom left shows all of the outputs for the selected point of the analysis.  

![stability analysis 14](../../../assets/user_manual_assets/xflr5%20pictures/stability%20analysis%2014.svg)







### Reading Graphs


#### Overview


On the home screen, you will see the analysis has updated. On the left you can select specific points of the analysis. In the middle you will see charts visualizing the analysis data. You can move between visualization by using the plot menu at the top.  

![reading graphs 1](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%201.svg)
#### OpPoint Viewer


![reading graphs 2](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%202.svg)
The OpPoint viewer shows several charts that display spanwise aerodynamic data at specific points of the analysis (e.g. at specific angles of attack)  

#### Polar Viewer


The polar viewer shows aerodynamic trends across the whole analysis and specifically how they relate to each other (e.g. how lift coefficient relates to angle of attack)  

![reading graphs 3](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%203.svg)
#### Root Locus Viewer


The root locus viewer shows the roots of the aircraft response to disturbances.  

![reading graphs 4](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%204.svg)
#### Time Response Viewer


The time response viewer shows the aircraft response to disturbances as a function of time.  

![reading graphs 5](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%205.svg)
#### Pressure Viewer


The pressure viewer shows the pressure distributions across the lifting surfaces of the wing at specific operating points of the analysis.  

![reading graphs 6](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%206.svg)
#### Changing Graphs


To change any of the graphs, right click on the graph, then select "Current Graph" and "Define Graph Settings". This will allow you to customize the axes and any other graph information.  

![reading graphs 7](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%207.svg)
#### Customizing Visuals


To change the lines on the graphs (color, weight, etc) right click on the airfoil or operating point on the list on the left then select your desired line characteristics.  

![reading graphs 8](../../../assets/user_manual_assets/xflr5%20pictures/reading%20graphs%208.svg)


















### Common Errors


#### Outside Flight Envelope


Cause: Your batch analysis includes angle of attack (α), Reynolds number (Re), or Mach number values beyond the range where data has been defined or computed for your airfoil(s).  

  

 To Fix:  

 - Go to the "Batch Analysis" settings.  

 - Restrict the angle of attack and Reynolds number range to within what you've previously analyzed in the airfoil polar analysis (e.g., -5° to +15°, 50,000 ≤ Re ≤ 1,000,000).  

 - Make sure your aircraft's velocity, altitude, and configuration don't push the operating point outside of what was analyzed.  

 - Expand your airfoil batch analysis or restrict the plane analysis until they are both consistent.


![common errors 1](../../../assets/user_manual_assets/xflr5%20pictures/common%20errors%201.svg)
#### Can't be Interpolated


 Cause: XFLR5 failed to interpolate a value because a requested parameter (e.g., lift coefficient or moment) is between or outside available data points.  

  

 To Fix:  

 - Check that your airfoil polar data is dense enough. You might need more data points at smaller intervals in AoA or Re.  

 - Re-run your airfoil analysis with finer steps, e.g., AoA from -6° to 16° in 0.5° increments.  

 - Ensure matching Reynolds numbers between your airplane analysis and airfoil polar data.
![common errors 2](../../../assets/user_manual_assets/xflr5%20pictures/common%20errors%202.svg)
#### Negative Lift


Cause: at the angle of attack analyzed, the airfoil does not have positive lift (it falls)  

  

 To Fix:  

 - Check the wing incidence angle and orientation in your model are correct.  

 - Check that your range of tested values in your analysis are appropriate.  

  

 If the above test reveal no problems, then this probably is not a problem and simply identifies an angle of attack that your aircraft cannot sustain.


![common errors 3](../../../assets/user_manual_assets/xflr5%20pictures/common%20errors%203.svg)


### References List and Helpful Links


* [Xflr5 Documentation](https://www.xflr5.tech/xflr5.htm)
* [Xflr5 Documentation (PDF)](https://aero.us.es/adesign/Slides/Extra/Aerodynamics/Software/XFLR5/XFLR5%20v6.10.02/Guidelines.pdf)
* [Stability Analysis Documentation and Tutorials](https://ebrary.net/59623/engineering/analyzing_decode_with_xflr5_stability)
* [Stability Analysis Documentation and Tutorials (flow5)](https://flow5.tech/docs/flow5_doc/Tutorials/Stability.html)
* [Stability Analysis Documentation and Tutorials (PDF)](https://www.xflr5.com/docs/XFLR5_and_Stability_analysis.pdf)
* [Publications Using Xflr5](https://www.researchpublish.com/upload/book/Lateral%20and%20Longitudinal%20Stability%20Analysis%20of%20UAV%20Using%20Xflr5-1163.pdf)
* [Tutorial Videos](https://www.youtube.com/playlist?list=PLtl5ylS6jdP6uOxzSJKPnUsvMbkmalfKg)






![OpenVSP Logo](../../../assets/user_manual_assets/pictures/OpenVSP_Logo.svg)
## OpenVSP




**Quick Navigation:**
* [Navigation](#navigation)
	+ [File Tab](#file-tab)
	+ [Edit Tab](#edit-tab)
	+ [Window Tab](#window-tab)
	+ [View Tab](#view-tab)
	+ [Model Tab](#model-tab)
	+ [Analysis Tab](#analysis-tab)
* [Making a Model](#making-a-model)
	+ [Wings](#wings)
	+ [Fuselage](#fuselage)
	+ [Using Reference Background Images](#using-reference-background-images)
* [Aircraft Analysis](#aircraft-analysis)
	+ [Basic Analysis](#basic-analysis)
	+ [VSPAERO Stability Analysis](#vspaero-stability-analysis)
	+ [Parasitic Drag](#parasitic-drag)
	+ [Mass Properties](#mass-properties)
* [Common Errors](#common-errors)
	+ [Meshing Problems](#meshing-problems)
* [References List and Helpful Links](#references-list-and-helpful-links)




### Navigation


When OpenVSP first opens on your computer, you will see an empty screen with a menu up top. The following slides will go through important menu options.


![navigation 1](../../../assets/user_manual_assets/openvsp%20pictures/navigation%201.svg)
#### File Tab


The "File" tab contains navigation options to save, export, and open projects. In the "Open VSP file" box, navigate to the file you want to open and click "Accept."


![navigation 2](../../../assets/user_manual_assets/openvsp%20pictures/navigation%202.svg)
#### Edit Tab


The "Edit" tab provides useful tools for setting parameter values during model creation and analysis. The most useful option in the "Edit" tab is the "Undo Parameter Change". Use this to return to previous parameter values during aircraft design and analysis.


![navigation 3](../../../assets/user_manual_assets/openvsp%20pictures/navigation%203.svg)
#### Window Tab


The "Window" tab allows you to customize the viewing windows. The "Background" tool is especially useful when creating aircraft models. Import a picture or drawing of an aircraft to help with dimensioning or relative positioning. The "Background" button allows you to import images into the background to aid in aircraft dimensioning and accurate modeling.


![navigation 4](../../../assets/user_manual_assets/openvsp%20pictures/navigation%204.svg)
#### View Tab


The "View" tab allows you to easily change the orientation of your model in the viewer.


![navigation 5](../../../assets/user_manual_assets/openvsp%20pictures/navigation%205.svg)
#### Model Tab


The "Model" tab contains useful tools for creating a model. Click on the "Geometry" button to open the geometry browser. Select the "Geometry" button to open the Geometry browser. This is where every component of your model will be created and stored. The Geometry Browser contains a variety of useful modeling tools. These will be discussed in more detail in the modeling section.


![navigation 6](../../../assets/user_manual_assets/openvsp%20pictures/navigation%206.svg)
#### Analysis Tab


In the "Analysis" tab you can find several helpful analysis tools for performing mass properties, drag, aerodynamic, and stability analyses. Use "Mass Prop" to find CG and moment of inertia calculations. Use VSPAERO for all aerodynamic and stability analyses. Use Parasite Drag for the most accurate drag estimates of your model.


![navigation 7](../../../assets/user_manual_assets/openvsp%20pictures/navigation%207.svg)


### Making a Model


#### Wings


In the "Model" tab, select the "Geometry" tool to open the model geometry browser.  

![making a model wings 1](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%201.svg)
 Select the geometry creation tool by clicking the drop down menu at the top of the window. After selecting your geometry item, click "add".  

![making a model wings 2](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%202.svg)
 After adding a geometry item, the geometry editor tool will open and allow you to customize the geometry.  

![making a model wings 3](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%203.svg)
 The general tab allows you to name your items, customize material and color properties, and assign them to specific geometry subsets.  

  

 Use the Xform tab to define the position, orientation, and other general characteristics of the wing.  

![making a model wings 4](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%204.svg)
 Define the Mass and physical properties of the item in the "Mass" tab.  

![making a model wings 5](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%205.svg)
 In the "Sub" tab, define subsections of the wing, such as control surfaces.  

![making a model wings 6](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%206.svg)
 After creating a control surface, a "sub surface parameters" section will appear. Use these tools to define the control surface position, size, functionality etc.  

![making a model wings 7](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%207.svg)
 In the "Plan" tab, define planform dimensions and other orientation details.  

![making a model wings 8](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%208.svg)
 For each section of the wing, define the size dimensions.  

![making a model wings 9](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%209.svg)
 Import airfoils and define airfoils for each wing section in the airfoil tab.  

![making a model wings 10](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%2010.svg)
 The Blending and Modify tabs have tools to blend airfoils together and edit the edge characteristics. These are more technical options that are outside of the scope of this manual.


![making a model wings 11](../../../assets/user_manual_assets/openvsp%20pictures/making%20a%20model%20wings%2011.svg)
#### Fuselage


Select the geometry creation tool by clicking the drop-down menu at the top of the window. After selecting the "fuselage" geometry item, click "add".  

![fuselage 1](../../../assets/user_manual_assets/openvsp%20pictures/fuselage%201.svg)
 The "Xform", "Mass", and "Sub" tabs are very similar to those for wings. Use these tabs to define the position, orientation, mass, and substructures of the fuselage.  

![fuselage 2](../../../assets/user_manual_assets/openvsp%20pictures/fuselage%202.svg)
 Go through these tabs to define the position, orientation, mass, and substructures of the fuselage.  

 In the "Design" tab, define the main characteristics of your fuselage.  

![fuselage 3](../../../assets/user_manual_assets/openvsp%20pictures/fuselage%203.svg)
 Define the length of your fuselage and other characteristics of the shape.  

 In the "Xsec" tab, you can customize the position, orientation, and shape of each section of the fuselage.  

![fuselage 4](../../../assets/user_manual_assets/openvsp%20pictures/fuselage%204.svg)
 In the "Skinning" tab, you can adjust the shape and curves of the outer surfaces on the fuselage.  

![fuselage 5](../../../assets/user_manual_assets/openvsp%20pictures/fuselage%205.svg)
 The "Modify" tab allows you to further edit the sections of the fuselage by adding "chevron" shapes (used to define more complex shape contours).  

![fuselage 6](../../../assets/user_manual_assets/openvsp%20pictures/fuselage%206.svg)
#### Using Reference Background Images


When modeling an aircraft, OpenVSP allows you to add a reference background image to help with modeling your aircraft correctly.  

 > Window > Background  

![background 1](../../../assets/user_manual_assets/openvsp%20pictures/background%201.svg)
 In the "Background" pop-up window, make sure the image box is checked. Then, click on the three dots next to the "File" box to select the image you want to import.  

![background 2](../../../assets/user_manual_assets/openvsp%20pictures/background%202.svg)
 Select the image you want to import and click the "Accept" button.  

![background 3](../../../assets/user_manual_assets/openvsp%20pictures/background%203.svg)
 After your background image is imported, align your model with it to create accurate dimensions for your model.  

![background 4](../../../assets/user_manual_assets/openvsp%20pictures/background%204.svg)
 Multiple images can be imported to aid in building your model from all angles. For instance, import top, side, and front view images to make sure your model is accurate from all perspectives.






### Aircraft Analysis


#### Basic Analysis


To run a basic aerodynamic analysis:  

 > Analysis (menu) > Aero > VSPAERO  

  

 A basic aerodynamic analysis computes the aircraft's aerodynamic state at a variety of positions and angles of attack. This is very useful for understanding the basic characteristics (lift, drag, static margin) of the aircraft. It does not involve any control surfaces and thus does not compute stability and control derivatives.  

![basic analysis 1](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%201.svg)
 Click on the "Advanced" tab at the top and define the analysis settings.  

![basic analysis 2](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%202.svg)
 In the "Overview" tab, define the rest of the analysis settings, then click "Launch Solver" at the bottom.  

![basic analysis 3](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%203.svg)
 When the analysis is finished, a results manager tab will open. On the "Load Dist." tab, you can see charts portraying a variety of aerodynamic coefficients across the span of the wing.  

![basic analysis 4](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%204.svg)
 The "Convergence" tab shows the precision of the coefficient calculations.  

![basic analysis 5](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%205.svg)
 The "Sweep" tab allows you to compare relationships between aerodynamic coefficients.  

![basic analysis 6](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%206.svg)
 After an analysis is completed, OpenVSP will produce several files with information with the analysis outputs. These files will appear in the same folder that you saved your original project in. The most useful file produced from the Basic Analysis is the "Polar" file that contains each coefficient calculated at each point of the analysis sweep.  

![basic analysis 7](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%207%20.svg)
 After opening the polar file, you will see a list of all the analysis iterations (each point along the defined sweep) and the corresponding aerodynamic coefficient outputs.  

![basic analysis 8](../../../assets/user_manual_assets/openvsp%20pictures/basic%20analysis%208.svg)
 In my analysis, I defined the sweep from 0-5 deg angle of attack. A useful analysis could be looking at the change in lift coefficient with alpha.


#### VSPAERO Stability Analysis


A stability analysis allows you to calculate the stability and control derivatives of your aircraft.  

 > Analysis > Aero > VSPAERO  

![stability analysis 1](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%201.svg)
 In the "Overview" tab, set up the main settings of the analysis.  

  

 You should actuate only one group of control surfaces per stability analysis. For this reason, you will most likely need to run several stability analyses.  

  

 Example: have one stability analysis with ailerons activated to get the aileron control derivatives, and another analysis to get the elevator control derivatives.  

![stability analysis 2](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%202.svg)
 In the Advanced tab, make sure the run mode is set to "Steady" and the advanced flow conditions are appropriate for your flight situation. (default units are in SI)  

![stability analysis 3](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%203.svg)
 In the "Control Grouping" tab, select specific control surfaces to assign to control groups for the analysis.  

![stability analysis 4](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%204.svg)
 After defining all of the analysis settings, return to the "Overview" tab and click the "Launch Solver" button at the bottom of the window.  

![stability analysis 5](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%205.svg)
 When the analysis is finished, a results manager tab will open. On the "Load Dist." tab, you can see charts portraying a variety of aerodynamic coefficients across the span of the wing.  

![stability analysis 6](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%206.svg)
 The "Convergence" tab shows the precision of the coefficient calculations.  

![stability analysis 7](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%207.svg)
 The "Sweep" tab allows you to compare relationships between aerodynamic coefficients.  

![stability analysis 8](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%208.svg)
 After an analysis is completed, OpenVSP will produce several files containing the analysis outputs. These files will appear in the same folder that you saved your original project in. The most useful files produced from the stability analysis are the (.stab) file and the (.flt) file.  

![stability analysis 9](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%209.svg)
 The .stab file contains all of the main aircraft dimensions and the dimensional and non-dimensional stability derivatives of the aircraft.  

![stability analysis 10](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%2010.svg)
 The .flt file contains the stability derivatives arranged in a more explicitly defined list. The values are the same as the .stab file; however, the CD\_o value is different. This seems to be the more accurate drag value, and may include other sources of drag not accounted for in the .stab file.  

![stability analysis 11](../../../assets/user_manual_assets/openvsp%20pictures/stability%20analysis%2011.svg)
#### Parasitic Drag


Parasitic drag can be analyzed using the drag analysis tool in the Analysis tab. This tool allows you to estimate the drag produced by the aircraft's components.  

 > Analysis > Aero > Parasite Drag  

![parasitic drag 1](../../../assets/user_manual_assets/openvsp%20pictures/parasitic%20drag%201.svg)
 
 Make sure the analysis settings are accurate, then calculate the drag.  

![parasitic drag 2](../../../assets/user_manual_assets/openvsp%20pictures/parasitic%20drag%202.svg)
#### Mass Properties


The mass properties tool provides information about the mass distribution and center of gravity of your model.  

 > Analysis > Mass Prop.  

![mass properties 1](../../../assets/user_manual_assets/openvsp%20pictures/mass%20properties%201.svg)
 Click the "Compute" button and the mass and inertia values should appear in the boxes below.  

![mass properties 2](../../../assets/user_manual_assets/openvsp%20pictures/mass%20properties%202.svg)
 After performing a mass properties analysis, the slidesd model of the geometry will appear on the model viewer.  

  

 To go back to viewing the original model, select "MeshGeom" in the geometry list and then click either "NoShow" or "Delete".  

  

 Then drag select all of the geometry parts of your aircraft and click "Show".  

![mass properties 3](../../../assets/user_manual_assets/openvsp%20pictures/mass%20properties%203.svg)








### Common Errors


#### Meshing Problems


If you encounter meshing problems, use the mesh diagnostics tool to identify and resolve issues with the mesh.  

  

 OpenVSP uses a panel and vortex lattice method to calculate the aerodynamic forces, moments, and coefficients. If there are any problems with the meshing of your model, these calculations may result in extreme values.  

  

 Any mesh problems can be identified by asymptotic behavior on the plots.  

![meshing problems 1](../../../assets/user_manual_assets/openvsp%20pictures/meshing%20problems%201.svg)




### References List and Helpful Links


* [OpenVSP Documentation](https://vspu.larc.nasa.gov/training-content/chapter-5-advanced-openvsp-techniques/)
* [Google groups](https://groups.google.com/g/openvsp/c/Pb93zCFufJA/m/mRHC0fD7CQAJ)
* [Example Videos](https://www.bing.com/videos/riverview/relatedvideo?q=using+openvsp+to+find+stability+derivatives&mid=797311886965F9F42A1A797311886965F9F42A1A&FORM=VIRE)
* [Example Research Papers (Barrella)](http://wpage.unina.it/danilo.ciliberti/doc/Barrella.pdf)
* [Example Research Papers (Vincenti)](http://wpage.unina.it/danilo.ciliberti/doc/Vincenti.pdf)
* [Example Research Papers (VT)](https://vtechworks.lib.vt.edu/server/api/core/bitstreams/db1aa291-5e0f-446a-bf5a-6d5e570a2719/content)





## Appendix


### Quick-Links


* [Aerodynamics Crash Course](#aerodynamics-crash-course)
* [How to Find the Best Stability and Control Derivatives](#how-to-find-the-best-stability-and-control-derivatives)
* [Notation Glossary](#notation-glossary)
* [Naming Conventions](#naming-conventions)
* [References, Helpful Links, and Documentation](#references-helpful-links-and-documentation)



### Aerodynamics Crash Course


* [Cm/Alpha Rules](#cm-alpha-rules)
* [Finding the Neutral Point with a Cm Chart](#neutral-point)
* [Static Margin](#static-margin)
* [Finding Steady Flight Speed](#flight-speed)
* [Finding Trimmed State](#aircraft-trimming)
* [Glide Ratio](#glide-ratio)
* [Root Locus Stability Charts](#root-locus-stability)


#### Cm/Alpha Stability


The Cm (center of mass) vs. Alpha curve must have a negative slope to be stable. It also should cross the positive x-axis to be stable.  

  

 If the slope is not positive or if the line does not cross the positive x-axis, then the aircraft will not be able to naturally hold level flight.  

![aerodynamics 1](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%201.svg)
#### Neutral Point


The neutral point (NP) is the aerodynamic center of the entire aircraft — the point along the longitudinal axis where the pitching moment doesn't change with angle of attack.  

  

 If the center of gravity (CG) is ahead of the NP, the aircraft is statically stable.  

 If the CG is behind the NP, the aircraft is unstable.  

![neutral point formula](../../../assets/user_manual_assets/appendix%20pictures/neutral%20point%20formula.svg)
  

 In Xflr5, you can find the calculated neutral point in the plane viewer tab after performing a basic aerodynamic analysis. You can also find the NP in the coefficient viewer window below the list of analyses after a stability analysis.  

  

 In OpenVSP, the neutral point can be found at the bottom right of the .stab file after a stability analysis is completed.  

![aerodynamics 3](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%203.svg)
 The neutral point can also be found in both Xflr5 and OpenVSP by finding the CG location at which the Cm vs. Alpha curve has a flat slope.  

![aerodynamics 4](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%204.svg)



#### Static Margin


Static margin is a measure of how stable an aircraft is. A higher SM means the aircraft will more quickly stabilize itself from disturbances, which usually also results in a lower maneuverability.  

  

 The static margin for most commercial and hobbyist aircraft is between 5-15%. 5-10% is more maneuverable, and 10-20% is more stable.  

![aerodynamics 5](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%205.svg)



#### Flight Speed


The y-intercept of a velocity vs. alpha chart will show you the steady flight speed at any alpha.  

![aerodynamics 6](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%206.svg)



#### Aircraft Trimming


A Cm vs. alpha chart x-intercept shows the trimmed angle of an aircraft.  

![aerodynamics 7](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%207.svg)



#### Glide Ratio


A glide ratio compares how far an aircraft can fly compared to how far it will fall.  

  

 The CL/CD vs. Cm chart y-intercept shows the glide ratio.  

![aerodynamics 8](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%208.svg)



#### Root Locus Stability


The root locus chart in the Xflr5 viewer shows the stability response of an aircraft.  

  

 The roots show stable / oscillatory characteristics.  

  

 If the roots are farther left, the aircraft is more stable. Farther from the x-axis = more oscillation.  

![aerodynamics 9](../../../assets/user_manual_assets/appendix%20pictures/aerodynamics%209.svg)



### How to Find the Best Stability and Control Derivatives


You should actuate only one group of control surfaces per stability analysis. For this reason, you will most likely need to run several stability analyses.  

  

 Example: have one stability analysis with ailerons activated to get the aileron control derivatives, and another analysis to get the elevator control derivatives.  

![finding derivatives 1](../../../assets/user_manual_assets/appendix%20pictures/finding%20derivatives%201.svg)
 The .flt file contains all of the stability derivatives, but does not contain the control derivatives. The drag coefficients in here seem to be the most accurate.  

![finding derivatives 2](../../../assets/user_manual_assets/appendix%20pictures/finding%20derivatives%202.svg)
  

 In Xflr5, the stability and control derivatives can be found in the output viewer after a stability analysis is completed.  

  

 Select an analysis from the list at the top left and the derivatives will appear below.  

![finding derivatives 3](../../../assets/user_manual_assets/appendix%20pictures/finding%20derivatives%203.svg)



### Notation Glossary


[Example Cessna 172Notation Glossary Link](https://us1mirror.flightgear.org/terrasync/fgdata/fgdata_2020_3/Aircraft-uiuc/models/cessna172/nonlinear.html)


![notation glossary 1](../../../assets/user_manual_assets/appendix%20pictures/notation%20glossary%201.svg)
### Naming Conventions


![naming conventions 1](../../../assets/user_manual_assets/appendix%20pictures/naming%20conventions%201.svg)
### References, Helpful Links, and Documentation


* [Online NACA Databases](http://airfoiltools.com/search/index?m%5Bgrp%5D=naca4d&m%5Bsort%5D=1)
* [NACA Database 2](https://m-selig.ae.illinois.edu/ads/coord_database.html)
* [Aircraft Dynamic Models Database](https://m-selig.ae.illinois.edu/apasim/Aircraft-uiuc.html)
* [NASA Aircraft Analysis Paper](https://ntrs.nasa.gov/api/citations/19720018355/downloads/19720018355.pdf)
* [How to Find Inertia Tensors for a Small Aircraft](https://www.schoolphysics.co.uk/age16-19/Mechanics/Simple%20harmonic%20motion/text/Bifilar_suspension/index.html)
* [Stability and Control Crash Course](https://ocw.mit.edu/courses/16-333-aircraft-stability-and-control-fall-2004/99dac83da0ad7eb8906ebac40e9e6ae1_lecture_2.pdf)
* [Flight Vehicle and Aerodynamic Design Textbook](https://flowlab.groups.et.byu.net/me415/flight.pdf)
* [Xflr5 Documentation](https://www.xflr5.tech/xflr5.htm)
* [Xflr5 Documentation (PDF)](https://aero.us.es/adesign/Slides/Extra/Aerodynamics/Software/XFLR5/XFLR5%20v6.10.02/Guidelines.pdf)
* [OpenVSP Documentation](https://vspu.larc.nasa.gov/training-content/chapter-5-advanced-openvsp-techniques/)
* [OpenVSP Google groups](https://groups.google.com/g/openvsp/c/Pb93zCFufJA/m/mRHC0fD7CQAJ)
* [OpenVSP Example Videos](https://www.bing.com/videos/riverview/relatedvideo?q=using+openvsp+to+find+stability+derivatives&mid=797311886965F9F42A1A797311886965F9F42A1A&FORM=VIRE)
* [Xflr5 Tutorial Videos](https://www.youtube.com/playlist?list=PLtl5ylS6jdP6uOxzSJKPnUsvMbkmalfKg)
* [Xflr5 Stability Analysis Documentation and Tutorials](https://ebrary.net/59623/engineering/analyzing_decode_with_xflr5_stability)
* [Stability Analysis Documentation and Tutorials (flow5)](https://flow5.tech/docs/flow5_doc/Tutorials/Stability.html)
* [Stability Analysis Documentation and Tutorials (PDF)](https://www.xflr5.com/docs/XFLR5_and_Stability_analysis.pdf)
* [Research papers using Xflr5 and/or OpenVSP (Barrella)](http://wpage.unina.it/danilo.ciliberti/doc/Barrella.pdf)
* [Research papers using Xflr5 and/or OpenVSP (Vincenti)](http://wpage.unina.it/danilo.ciliberti/doc/Vincenti.pdf)
* [Research papers using Xflr5 and/or OpenVSP (VT)](https://vtechworks.lib.vt.edu/server/api/core/bitstreams/db1aa291-5e0f-446a-bf5a-6d5e570a2719/content)
* [Research papers using Xflr5 and/or OpenVSP (UAV)](https://www.researchpublish.com/upload/book/Lateral%20and%20Longitudinal%20Stability%20Analysis%20of%20UAV%20Using%20Xflr5-1163.pdf)
* [Research papers using Xflr5 and/or OpenVSP (Cessna 172N)](https://www.researchgate.net/publication/373635436_Model_Based_Aircraft_Design_and_Optimization_A_Case_Study_with_Cessna_172N_Aircraft)
* [ICAS 2022 Paper Comparing Xflr5 and OpenVSP](https://www.icas.org/icas_archive/ICAS2022/data/papers/ICAS2022_0367_paper.pdf)











You can also download this user manual as a single file and then open it in your browser to render it.

[Google Drive link to file](https://drive.google.com/file/d/10-37yCIjK796dnT-5MB5-q3M8o2nCi5_/view)
