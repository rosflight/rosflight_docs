
This section contains an aerodynamics crash course, common errors, notation and nomenclature outlines, and additional useful resources/links for further study. 

## Quick-Links


* [Aerodynamics Crash Course](#aerodynamics-crash-course)
* [How to Find the Best Stability and Control Derivatives](#how-to-find-the-best-stability-and-control-derivatives)
* [Notation Glossary](#notation-glossary)
* [References, Helpful Links, and Documentation](#references-helpful-links-and-documentation)



## Aerodynamics Crash Course


* [Cm/Alpha Rules](#cmalpha-stability)
* [Finding the Neutral Point with a Cm Chart](#neutral-point)
* [Static Margin](#static-margin)
* [Finding Steady Flight Speed](#flight-speed)
* [Finding Trimmed State](#aircraft-trimming)
* [Glide Ratio](#glide-ratio)
* [Root Locus Stability Charts](#root-locus-stability)


### Cm/Alpha Stability


The Cm (center of mass) vs. Alpha curve must have a negative slope to be stable. It also should cross the positive x-axis to be stable.  

  

 If the slope is not positive or if the line does not cross the positive x-axis, then the aircraft will not be able to naturally hold level flight.  

![aerodynamics 1](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%201.jpeg)
### Neutral Point


The neutral point (NP) is the aerodynamic center of the entire aircraft — the point along the longitudinal axis where the pitching moment doesn't change with angle of attack.  

  

 If the center of gravity (CG) is ahead of the NP, the aircraft is statically stable.  

 If the CG is behind the NP, the aircraft is unstable.  

![neutral point formula](../../assets/user_manual_assets/appendix%20pictures/neutral%20point%20formula.jpeg)
  

 In Xflr5, you can find the calculated neutral point in the plane viewer tab after performing a basic aerodynamic analysis. You can also find the NP in the coefficient viewer window below the list of analyses after a stability analysis.  

  

 In OpenVSP, the neutral point can be found at the bottom right of the .stab file after a stability analysis is completed.  

![aerodynamics 3](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%203.jpeg)
 The neutral point can also be found in both Xflr5 and OpenVSP by finding the CG location at which the Cm vs. Alpha curve has a flat slope.  

![aerodynamics 4](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%204.jpeg)



### Static Margin


Static margin is a measure of how stable an aircraft is. A higher SM means the aircraft will more quickly stabilize itself from disturbances, which usually also results in a lower maneuverability.  

  

 The static margin for most commercial and hobbyist aircraft is between 5-15%. 5-10% is more maneuverable, and 10-20% is more stable.  

![aerodynamics 5](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%205.jpeg)



### Flight Speed


The y-intercept of a velocity vs. alpha chart will show you the steady flight speed at any alpha.  

![aerodynamics 6](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%206.jpeg)



### Aircraft Trimming


A Cm vs. alpha chart x-intercept shows the trimmed angle of an aircraft.  

![aerodynamics 7](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%207.jpeg)



### Glide Ratio


A glide ratio compares how far an aircraft can fly compared to how far it will fall.  

  

 The CL/CD vs. Cm chart y-intercept shows the glide ratio.  

![aerodynamics 8](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%208.jpeg)



### Root Locus Stability


The root locus chart in the Xflr5 viewer shows the stability response of an aircraft.  

  

 The roots show stable / oscillatory characteristics.  

  

 If the roots are farther left, the aircraft is more stable. Farther from the x-axis = more oscillation.  

![aerodynamics 9](../../assets/user_manual_assets/appendix%20pictures/aerodynamics%209.jpeg)



## How to Find the Best Stability and Control Derivatives


You should actuate only one group of control surfaces per stability analysis. For this reason, you will most likely need to run several stability analyses.  

  

 Example: have one stability analysis with ailerons activated to get the aileron control derivatives, and another analysis to get the elevator control derivatives.  

![finding derivatives 1](../../assets/user_manual_assets/appendix%20pictures/finding%20derivatives%201.jpeg)
 The .flt file contains all of the stability derivatives, but does not contain the control derivatives. The drag coefficients in here seem to be the most accurate.  

![finding derivatives 2](../../assets/user_manual_assets/appendix%20pictures/finding%20derivatives%202.jpeg)
  

 In Xflr5, the stability and control derivatives can be found in the output viewer after a stability analysis is completed.  

  

 Select an analysis from the list at the top left and the derivatives will appear below.  

![finding derivatives 3](../../assets/user_manual_assets/appendix%20pictures/finding%20derivatives%203.jpeg)



## Notation Glossary
This section contains a glossary of common names and abbreviated notation for the most common stability and control derivatives used when analyzing aircraft. 

It also lists and compares the stability and control derivative naming conventions for each tool discussed in this manual (ROSplane, Xflr5, and OpenVSP). 

!!! note 
    ROSplane allows users to include non-standard stability and control derivatives that are not commonly used in aircraft analysis. These, along with helpful tips, are located under the "non-standard" section at the bottom of the picture. 

!!! danger ""C_D_p" in ROSflight is the parasitic drag coefficient!"
    In the ROSplane parameters, C_D_p is "parasitic drag", **not** drag from roll rate. Make sure to take this into account when building your model and inputting values into your .yaml file. Instructions for calculating parasitic drag can be found [here](./user-manual-openvsp.md#parasitic-drag).

![naming conventions 1](../../assets/user_manual_assets/appendix%20pictures/naming%20conventions%20picture.jpg)

The following tables define the key equations and variables that are used to calculate the stability and control derivatives referenced above. The equations are from *Airplane Flight Dynamics and Automatic Flight Controls* by J. Roskam.



<style>
table th, table td {
  padding: 2px 6px !important;
  line-height: 1.1em !important;
}
</style>

<small>
<div align="left" style="margin-bottom:0.1em;"><h4>Variable Definitions</h4></div>

| **Symbol** | **Description** | **Units** |
|:--|:--|:--|
| $\alpha$ | Angle of attack | rad |
| $\beta$ | Angle of sideslip | rad |
| $p, q, r$ | Roll, pitch, and yaw rates | rad/s |
| $b$ | Wing span | m |
| $\bar{c}$ | Mean aerodynamic chord | m |
| $U_1$ | Freestream (reference) airspeed | m/s |
| $\delta_e, \delta_a, \delta_r$ | Elevator, aileron, and rudder deflections | rad |
| $\dfrac{q\bar{c}}{2U_1}$ | Nondimensional pitch rate | — |
| $\dfrac{pb}{2U_1}$ | Nondimensional roll rate | — |
| $\dfrac{rb}{2U_1}$ | Nondimensional yaw rate | — |
| $C_L, C_D, C_Y$ | Lift, drag, and side-force coefficients | — |
| $C_\ell, C_m, C_n$ | Rolling, pitching, and yawing moment coefficients | — |

</small>

<div align="left" style="margin-bottom:0.1em;"><h4>Longitudinal Coefficients</h4></div>

<small>

| **Symbol** | **Formula** | **Description** | **Units** |
|:--|:--|:--|:--|
| $C_{L_0}$ | — | Lift coefficient at zero angle of attack | — |
| $C_{D_0}$ | — | Zero-lift drag coefficient | — |
| $C_{m_0}$ | — | Pitching moment coefficient at zero angle of attack | — |
| $C_{L_\alpha}$ | $\dfrac{\partial C_L}{\partial \alpha}$ | Variation of lift with angle of attack | 1/rad |
| $C_{D_\alpha}$ | $\dfrac{\partial C_D}{\partial \alpha}$ | Variation of drag with angle of attack | 1/rad |
| $C_{m_\alpha}$ | $\dfrac{\partial C_m}{\partial \alpha}$ | Variation of pitching moment with angle of attack | 1/rad |
| $C_{L_q}$ | $\dfrac{\partial C_L}{\partial (q\bar{c}/2U_1)}$ | Variation of lift with nondimensional pitch rate | 1/rad |
| $C_{D_q}$ | $\dfrac{\partial C_D}{\partial (q\bar{c}/2U_1)}$ | Variation of drag with nondimensional pitch rate | 1/rad |
| $C_{m_q}$ | $\dfrac{\partial C_m}{\partial (q\bar{c}/2U_1)}$ | Variation of pitching moment with nondimensional pitch rate | 1/rad |
| $C_{L_{\delta_e}}$ | $\dfrac{\partial C_L}{\partial \delta_e}$ | Variation of lift with elevator deflection | 1/rad |
| $C_{D_{\delta_e}}$ | $\dfrac{\partial C_D}{\partial \delta_e}$ | Variation of drag with elevator deflection | 1/rad |
| $C_{m_{\delta_e}}$ | $\dfrac{\partial C_m}{\partial \delta_e}$ | Variation of pitching moment with elevator deflection | 1/rad |

</small>

<div align="left" style="margin-bottom:0.1em;"><h4>Lateral/Directional Coefficients</h4></div>

<small>

| **Symbol** | **Formula** | **Description** | **Units** |
|:--|:--|:--|:--|
| $C_{Y_0}$ | — | Side force coefficient for zero sideslip and zero control deflection | — |
| $C_{\ell_0}$ | — | Rolling moment coefficient for zero sideslip and zero control deflection | — |
| $C_{n_0}$ | — | Yawing moment coefficient for zero sideslip and zero control deflection | — |
| $C_{Y_\beta}$ | $\dfrac{\partial C_Y}{\partial \beta}$ | Variation of side force with sideslip angle | 1/rad |
| $C_{\ell_\beta}$ | $\dfrac{\partial C_\ell}{\partial \beta}$ | Variation of rolling moment with sideslip angle | 1/rad |
| $C_{n_\beta}$ | $\dfrac{\partial C_n}{\partial \beta}$ | Variation of yawing moment with sideslip angle | 1/rad |
| $C_{Y_p}$ | $\dfrac{\partial C_Y}{\partial (pb/2U_1)}$ | Variation of side force with nondimensional roll rate | 1/rad |
| $C_{\ell_p}$ | $\dfrac{\partial C_\ell}{\partial (pb/2U_1)}$ | Variation of rolling moment with nondimensional roll rate | 1/rad |
| $C_{n_p}$ | $\dfrac{\partial C_n}{\partial (pb/2U_1)}$ | Variation of yawing moment with nondimensional roll rate | 1/rad |
| $C_{Y_r}$ | $\dfrac{\partial C_Y}{\partial (rb/2U_1)}$ | Variation of side force with nondimensional yaw rate | 1/rad |
| $C_{\ell_r}$ | $\dfrac{\partial C_\ell}{\partial (rb/2U_1)}$ | Variation of rolling moment with nondimensional yaw rate | 1/rad |
| $C_{n_r}$ | $\dfrac{\partial C_n}{\partial (rb/2U_1)}$ | Variation of yawing moment with nondimensional yaw rate | 1/rad |
| $C_{Y_{\delta_a}}$ | $\dfrac{\partial C_Y}{\partial \delta_a}$ | Variation of side force with aileron deflection | 1/rad |
| $C_{\ell_{\delta_a}}$ | $\dfrac{\partial C_\ell}{\partial \delta_a}$ | Variation of rolling moment with aileron deflection | 1/rad |
| $C_{n_{\delta_a}}$ | $\dfrac{\partial C_n}{\partial \delta_a}$ | Variation of yawing moment with aileron deflection | 1/rad |
| $C_{Y_{\delta_r}}$ | $\dfrac{\partial C_Y}{\partial \delta_r}$ | Variation of side force with rudder deflection | 1/rad |
| $C_{\ell_{\delta_r}}$ | $\dfrac{\partial C_\ell}{\partial \delta_r}$ | Variation of rolling moment with rudder deflection | 1/rad |
| $C_{n_{\delta_r}}$ | $\dfrac{\partial C_n}{\partial \delta_r}$ | Variation of yawing moment with rudder deflection | 1/rad |

</small>

<div align="left" style="margin-bottom:0.1em;"><h4>Cross-Coupled Derivatives</h4></div>

<small>

| **Symbol** | **Formula** | **Description** | **Units** |
|:--|:--|:--|:--|
| $C_{L_\beta}$ | $\dfrac{\partial C_L}{\partial \beta}$ | Variation of lift with sideslip angle | 1/rad |
| $C_{L_p}$ | $\dfrac{\partial C_L}{\partial (pb/2U_1)}$ | Variation of lift with nondimensional roll rate | 1/rad |
| $C_{L_r}$ | $\dfrac{\partial C_L}{\partial (rb/2U_1)}$ | Variation of lift with nondimensional yaw rate | 1/rad |
| $C_{L_{\delta_a}}$ | $\dfrac{\partial C_L}{\partial \delta_a}$ | Variation of lift with aileron deflection | 1/rad |
| $C_{L_{\delta_r}}$ | $\dfrac{\partial C_L}{\partial \delta_r}$ | Variation of lift with rudder deflection | 1/rad |
| $C_{D_\beta}$ | $\dfrac{\partial C_D}{\partial \beta}$ | Variation of drag with sideslip angle | 1/rad |
| $C_{D_p}$ | $\dfrac{\partial C_D}{\partial (pb/2U_1)}$ | Variation of drag with nondimensional roll rate | 1/rad |
| $C_{D_r}$ | $\dfrac{\partial C_D}{\partial (rb/2U_1)}$ | Variation of drag with nondimensional yaw rate | 1/rad |
| $C_{D_{\delta_a}}$ | $\dfrac{\partial C_D}{\partial \delta_a}$ | Variation of drag with aileron deflection | 1/rad |
| $C_{D_{\delta_r}}$ | $\dfrac{\partial C_D}{\partial \delta_r}$ | Variation of drag with rudder deflection | 1/rad |

</small>

!!! note
    All derivatives evaluated at reference condition (stability-axis system). Rate terms use nondimensional rates $\frac{pb}{2U_1}$, $\frac{q\bar{c}}{2U_1}$, $\frac{rb}{2U_1}$. Units: angle/control = 1/rad.

 

</small>



## References, Helpful Links, and Documentation


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









