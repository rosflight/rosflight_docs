# Integrating Aircraft Models into ROSflight

Before integrating your aircraft model into the ROSflight simulation, complete all of the required stability analyses. Go to these [Xflr5](./user-manual-xflr5.md#stability-analysis) and [OpenVSP](./user-manual-openvsp.md#vspaero-stability-analysis) sections for information on how to run a stability analysis. 

After running the necessary aerodynamic and stability analyses, the next step is to add the stability and control derivatives into the required parameter file. 

The file you need to change is a "dynamics.yaml" file located here: 
`rosflight_ros_pkgs/rosflight_sim/params/anaconda_dynamics.yaml`

A sample parameter file is provided below. 

??? tip "Sample parameter file" 
        # Parameters for ROSflight software-in-the-loop simulation, based on RMRC Anaconda UAV.
        # Authors: Ian Reid and Phil Tokumaru

        # Mass and inertia parameters are defined in fixedwing.urdf.xacro.
        # Mass: 4.5
        # Jx: 0.24855
        # Jy: 0.3784
        # Jz: 0.618
        # Jxz: 0.06

        /**:  # Apply to any node that we load it to
        ros__parameters:
            rho: 1.2682
            mass: 4.5

        /fixedwing_forces_and_moments:
        ros__parameters:
            wing_s: .52
            wing_b: 2.08
            wing_c: 0.2350
            wing_M: 50.0
            wing_epsilon: 0.9 # revisit -- may not need this for the calculation
            wing_alpha0: 0.05236
            
            D_prop : 0.381 # prop diameter in m (15 in)
            CT_0 : 0.06288743
            CT_1 : -0.02704452
            CT_2 : -0.31320732
            CQ_0 :  0.00614891
            CQ_1 : -0.0106795
            CQ_2 : -0.011779
            KV : 560.0        # Motor speed constant from datasheet in RPM/V
            KQ : 0.01705                       #((1 /(KV_rpm_per_volt)) *60) / (2 *M_PI) Back-emf constant, KV in V-s/rad, Motor torque constant, KQ in N-m/A
            V_max : 24.0                  # voltage for 6s battery at 4 volts per cell
            R_motor : 0.042              # ohms
            I_0 : 1.5                     # no-load (zero torque) current (A)

        servo_tau: .01
        servo_refresh_rate: 0.003
        max_aileron_deflection_angle: 40. # Maximum deflection of the control surfaces in degrees.
        max_rudder_deflection_angle: 40. # Maximum deflection of the control surfaces in degrees.
        max_elevator_deflection_angle: 40. # Maximum deflection of the control surfaces in degrees.

        # Anaconda done for C_L

        C_L_O: .506
        C_L_alpha: 5.61
        C_L_beta: 0.0
        C_L_p: 0.0
        C_L_q: 7.95
        C_L_r: 0.0
        C_L_delta_a: 0.0
        C_L_delta_e: 0.13
        C_L_delta_r: 0.0

        # Anaconda done for C_D

        C_D_O: 0.043
        C_D_alpha: 0.03
        C_D_beta: 0.0
        C_D_p: 0.043
        C_D_q: 0.0
        C_D_r: 0.0
        C_D_delta_a: 0.0
        C_D_delta_e: 0.0135
        C_D_delta_r: 0.0

        C_ell_O: 0.0
        C_ell_alpha: 0.00
        C_ell_beta: -0.13
        C_ell_p: -0.51
        C_ell_q: 0.0
        C_ell_r: 0.25
        C_ell_delta_a: 0.17
        C_ell_delta_e: 0.0
        C_ell_delta_r: 0.0024

        C_m_O: 0.135
        C_m_alpha: -2.74
        C_m_beta: 0.0
        C_m_p: 0.0
        C_m_q: -38.21
        C_m_r: 0.0
        C_m_delta_a: 0.0
        C_m_delta_e: -.99
        C_m_delta_r: 0.0

        C_n_O: 0.0
        C_n_alpha: 0.0
        C_n_beta: 0.1301
        C_n_p: -0.0364
        C_n_q: 0.0
        C_n_r: -0.1541
        C_n_delta_a: -0.011
        C_n_delta_e: 0.0
        C_n_delta_r: -0.069

        # Anaconda done for C_Y

        C_Y_O: 0.0
        C_Y_alpha: 0.00
        C_Y_beta: -0.98
        C_Y_p: 0.0
        C_Y_q: 0.0
        C_Y_r: 0.0
        C_Y_delta_a: 0.075
        C_Y_delta_e: 0.0
        C_Y_delta_r: .19

        /standalone_dynamics:
        ros__parameters:
            Jxx: 0.24855
            Jxy: 0.0
            Jxz: 0.06
            Jyy: 0.3784
            Jyz: 0.0
            Jzz: 0.618







The coefficients that need to be changed are the `C_L, C_D, C_m, C_Y, C_n, C_ell` derivatives. Replace the values for all of these stability and control derivatives with the values obtained from the stability and control analyses you ran with either Xflr5 or OpenVSP. 

!!! note
    Some of the derivatives are zero because they are "coupled" which means that they describe cross-axis effects. Usually these have a small effect on an aircraft and don't add much accuracy. See the Appendix section [notation glossary](./user-manual-appendix.md#notation-glossary) to see which coefficients are coupled. Use your own best judgement when deciding whether or not to include these. 

Once your parameter file is completed, save and re-name the file. You can then call this file when you launch the simulation. 

Steps to add a new aircraft to the simulation environment:

1. Complete your aircraft model and complete all required aerodynamic, stability, and control analyses
2. Create new .yaml parameter file and update coefficient values (example: `cessna_dynamics.yaml`)
3. Save .yaml file in `rosflight_ros_pkgs/rosflight_sim/params/`
4. Call your aircraft model (example: cessna) when running the simulation with:

    ```bash 
    ros2 launch rosflight_sim fixedwing_standalone.launch.py dynamics_param_file:=/path/to/param/file.yaml
    ```
