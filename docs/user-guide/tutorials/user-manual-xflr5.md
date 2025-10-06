

![Xflr5 Logo](../../../assets/user_manual_assets/pictures/Xflr5_Logo.svg)

**Quick Navigation:**

* [Navigation](#navigation)
    * [Direct Foil Design](#direct-foil-design)
    * [Xfoil Inverse Design](#xfoil-inverse-design)
    * [Xfoil Direct Analysis](#xfoil-direct-analysis)
    * [Wing and Plane Design](#wing-and-plane-design)
* [Creating and Analyzing Airfoils](#creating-and-analyzing-airfoils)
    * [Direct Foil Design](#direct-foil-design)
    * [Adding NACA Foils](#adding-naca-foils)
    * [Adding .dat Files](#adding-dat-files)
    * [Adding Flaps](#adding-flaps)
    * [Inverse Foil Design](#inverse-foil-design)
    * [XFoil Direct Analysis](#xfoil-direct-analysis)
* [Creating a Plane](#creating-a-plane)
    * [Information Checklist](#information-checklist)
    * [Making the Plane](#making-the-plane)
    * [Mass and Inertia Inputs](#mass-and-inertia-inputs)
* [Running an Analysis](#running-an-analysis)
    * [Basic Analysis](#basic-analysis)
    * [Stability Analysis](#stability-analysis)
* [Reading Graphs](#reading-graphs)
    * [Overview](#overview)
    * [OpPoint Viewer](#oppoint-viewer)
    * [Polar Viewer](#polar-viewer)
    * [Root Locus Viewer](#root-locus-viewer)
    * [Time Response Viewer](#time-response-viewer)
    * [Pressure Viewer](#pressure-viewer)
    * [Changing Graphs](#changing-graphs)
    * [Customizing Visuals](#customizing-visuals)
* [Common Errors](#common-errors)
    * [Outside Flight Envelope](#outside-flight-envelope)
    * [Can't be Interpolated](#cant-be-interpolated)
    * [Negative Lift](#negative-lift)
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

