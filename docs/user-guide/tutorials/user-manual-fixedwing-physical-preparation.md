# Fixed-Wing Modeling Preparation: Data Requirements & Pre-Modeling Checklist

This document describes the key parameters, measurements, and preparation steps required before modeling a fixed-wing aircraft in **OpenVSP** or **XFLR5**.  

!!! note "Important!"
    Identifying the necessary geometry and physical parameters for your aircraft is crucial for achieving accurate aerodynamic, stability, and control analyses.


## Pre-Modeling Checklist

Use this checklist to ensure you have all the necessary information to start modeling. 

### Geometry & Structure
- [ ] wingspan, root/tip chords, MAC, and sweep angles.
- [ ] Measure dihedral, incidence, and twist distribution across wing and tail.
- [ ] Identify wing taper ratio and control surface dimensions.
- [ ] Estimate tail areas and moment arms (horizontal and vertical).
- [ ] Acquire or digitize airfoil `.dat` files for root and tip of wing and tail. 
- [ ] Record all the position of every part of the aircraft relative to the wing leading edge.

### Mass & Balance
- [ ] Determine total aircraft mass (with payload if applicable).
- [ ] Measure or estimate CG location (x, y, z) from leading edge of the wing.
- [ ] Compute inertia tensors (I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub>, I<sub>xz</sub>).

### Propulsion & Power
- [ ] Collect propeller diameter, pitch, and manufacturer model.
- [ ] Obtain thrust–RPM data or perform static thrust test.
- [ ] Record motor Kv, max current, and expected RPM range.

### Flight Conditions
- [ ] Record expected airspeed, elevation, and air density
- [ ] Calculate expected Reynolds number and Mach number

### Reference & Export
- [ ] Compute S<sub>ref</sub>, b<sub>ref</sub>, and c<sub>ref</sub>.
- [ ] Define aerodynamic reference location (x<sub>ref</sub>, z<sub>ref</sub>).

---

## Calculating Inertia Values

Accurate **moments of inertia** are critical for stability and control simulation. If CAD mass data are not available, the following methods can be used.

### 1. Analytical Estimation by Component

Treat each major component as a simple geometric body with known mass `m`, length `L`, radius `r`, or span `b`.  
Then compute each part’s inertia about its own centroid and shift to the aircraft CG using the **Parallel Axis Theorem**:

\[
I_{CG} = I_{centroid} + m d^2
\]

where \( d \) = distance between component CG and total CG.

| Component | Model Shape | Approx. Inertia Formula (about centroid) |
|------------|--------------|-----------------------------------------|
| Wing | Rectangular plate | \( I_{xx} = \frac{1}{12} m b^2 \) |
| Fuselage | Solid cylinder | \( I_{yy} = \frac{1}{12} m (3r^2 + L^2) \) |
| Horizontal tail | Rectangular plate | \( I_{xx} = \frac{1}{12} m b_t^2 \) |
| Vertical tail | Thin plate | \( I_{zz} = \frac{1}{12} m h_t^2 \) |

Sum each component’s inertia about the same reference axes to obtain the total \( I_{xx}, I_{yy}, I_{zz} \), and any cross term \( I_{xz} \) if asymmetric.


### 2. Experimental Measurement

If the aircraft is built and accesssible, use a **bifilar pendulum test**:

1. Suspend the aircraft along an axis with two parallel strings.
2. Release the suspended aircraft (induce a twist) and measure the oscillation period \( T \).
3. Compute inertia using  
   \[
   I = \frac{m g R^2 T^2}{4 \pi^2 L}
   \]
   where  
   \( R \) = half distance between suspension lines,  
   \( L \) = string length.
4. Repeat for pitch, roll, and yaw axes to obtain \( I_{xx}, I_{yy}, I_{zz} \).

This website provies a brief overview of [bifilar pendulum testing](https://www.schoolphysics.co.uk/age16-19/Mechanics/Simple%20harmonic%20motion/text/Bifilar_suspension/index.html) to identify inertia values. 

---

## Required Aircraft Information (And where to find it)

| **Category** | **Needed Data** | **How to Obtain** |
|---------------|----------------|-------------------|
| **General Dimensions** | Wingspan, mean aerodynamic chord (MAC), root/tip chord, sweep, dihedral, taper ratio | From manufacturer specs, CAD drawings, physical measurement, or scaling from 3-view drawings. |
| **Airfoils** | Root and tip airfoil names, corresponding `.dat` files | Download from [UIUC Airfoil Database](https://m-selig.ae.illinois.edu/ads/coord_database.html), [airfoiltools.com](https://airfoiltools.com), or digitize using WebPlotDigitizer from reference images. |
| **Wing Planform** | Number of panels, twist distribution, control surface span/chord percentages | Obtain from RC plans, 3-view drawings, or measure from CAD. |
| **Tail Geometry** | Tail moment arm, horizontal and vertical tail area, aspect ratio | Estimate from scaled drawings or photographs in CAD/GIMP. |
| **Mass Properties** | Total mass, CG location, and inertia tensor (I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub>, I<sub>xz</sub>) | From CAD mass simulation, inertial pendulum test, or analytical estimation (see below). |
| **Propeller / Power** | Propeller diameter, RPM range, thrust curve | From manufacturer data or static thrust testing. |
| **Reference Values** | S<sub>ref</sub>, b<sub>ref</sub>, c<sub>ref</sub>, x<sub>ref</sub>, z<sub>ref</sub> | Look in aircraft documentation / manufacturing specifications or manually measure aircraft and calculate necesary values. |

---

Once you have all the information above, you are ready to model and analyse your aircraft!!
