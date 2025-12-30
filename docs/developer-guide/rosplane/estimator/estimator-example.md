# Estimator Continuous-Discrete

<!-- TODO: rename this to continuous discrete once full state is integrated. -->
<!-- TODO: Remove the cites and replace with something more appropriate -->

## Overview

The `EstimatorContinuousDiscrete` class implements a continuous-discrete Kalman filter as described in section 8.5 of the [UAV book](https://github.com/randybeard/mavsim_public) or 8.6 and 8.7 of volume one of the same book.
Specifically, this estimator uses the filter described in 8.11.
The estimator runs on a set timer with a configurable frequency (see Parameters section for details).
It estimates the position, velocities, attitude and gyroscope biases.

The state vector $\boldsymbol{x}$ is,
\begin{equation}
\boldsymbol{x} = 
\begin{bmatrix}
\boldsymbol{p} & \boldsymbol{v} & \boldsymbol{\theta} & \boldsymbol{b}_{gyro}
\end{bmatrix}^\top.
\end{equation}

The position of the body frame, $\boldsymbol{p}
The velocity of the body frame, $\boldsymbol{v}$, is expressed in the inertial frame also using NED coordinates in meters per second.
The attitude of the body frame, $\boldsymbol{\theta}$, relative to the inertial frame in radians using the Euler angles roll, pitch and yaw, $\phi,\,\theta,\,\psi$ respectively.
These use the ZYX convention of application as in \cite{uavbook}.
Finally, $\boldsymbol{b}_{gyro}$ is the bias in the gyroscope measurements expressed in the body frame in radians per second.

The attitude is modeled using Euler angles rather than a quaternion or other Lie group is for ease of interpretibility.
A major focus of ROScopter is understandability and extensibility and an Euler formulation helps facilitate the understandability of the estimator.
In addition, the results show that the performance is still excellent and robust despite the suboptimality of an Euler angle formulation.

The covariance of the state estimate is given as,
\begin{equation}
    P = E[\boldsymbol{x}^\top\boldsymbol{x}]
\end{equation}

### State Propagation
The state estimator for ROScopter is a continuous-discrete formulation as in \cite{uavbook}, meaning that the derivation and analysis utilizes the continuous time equations that are then developed into discrete time equations for use in the estimator.
This has the advantage of having a direct connection to the familiar dynamical equations and a clear relationship to the discrete time formulation that follows.

The dynamics of the aircraft state are modeled as a function of the state and the input to the system $\boldsymbol{u} = \begin{bmatrix} \boldsymbol{a} & \omega \end{bmatrix}$.

\begin{equation}
    \boldsymbol{f}(\boldsymbol{x}, \boldsymbol{u}) = \dot{\boldsymbol{x}} =
    \begin{bmatrix}
    R_b^i(\boldsymbol{\theta})\boldsymbol{v} \\ 
    R_b^i(\boldsymbol{\theta})^\top[0 & 0 & g]^\top + \boldsymbol{a} + \boldsymbol{v}\times\boldsymbol{\omega} \\
    S(\boldsymbol{\theta})\boldsymbol{\omega} \\
    \boldsymbol{0}_{3\times1}
    \end{bmatrix}.
\end{equation}

Since the actual inputs $\boldsymbol{a}$ and $\boldsymbol{\omega}$ are unavailable, we use the IMU to approximate these as $\boldsymbol{a} = \boldsymbol{y}_{accel}$ and $\boldsymbol{\omega} = \boldsymbol{y}_{gyro} - \boldsymbol{b}_{gyro}$.
Note that these are not integrated angles, but assumed to be the instantaneous measurement of the acceleration and angular rates.
The rotation matrix $R_b^i(\boldsymbol{\theta})$ expresses vectors in the body frame in inertial frame according to the current attitude estimate, $\boldsymbol{\theta}$.

The Jacobian of the dynamics with respect to the states is,

\begin{equation}
    A(\boldsymbol{x}, \boldsymbol{u}) = \begin{bmatrix}
        \boldsymbol{0}_{3\times3} & R_b^i(\boldsymbol{\theta}) & \frac{\partial R_b^i(\boldsymbol{\theta}) \boldsymbol{v}}{\partial \boldsymbol{\theta}} & \boldsymbol{0}_{3\times3} \\
        \boldsymbol{0}_{3\times3} & -[\boldsymbol{\omega}]_\times & \frac{\partial R_b^i(\boldsymbol{\theta})^\top \boldsymbol{g}}{\partial \boldsymbol{\theta}}  & \boldsymbol{0}_{3\times3} \\
        \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \frac{\partial S(\boldsymbol{\theta})\boldsymbol{\omega}}{\partial \boldsymbol{\theta}} & -S(\boldsymbol{\theta}) \\
        \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{I}_{3\times3} \\
    \end{bmatrix}.
\end{equation}

Note that this is the continuous time Jacobian not the discrete time Jacobian.
To find the discrete time Jacobian so it can be used in the EKF's discrete updates, we use a second order approximation of the matrix exponential, $e^{AT_s}$, where $T_s$ is the period between state propagation updates in the EKF.
It is given by, 

\begin{equation}
    A_d(\boldsymbol{x},\boldsymbol{u}) = I + A(\boldsymbol{x}, \boldsymbol{u})T_s + A(\boldsymbol{x}, \boldsymbol{u})^2\frac{T_s^2}{2}.
\end{equation}

The propagation step is split into $N$ updates.
This reduces linearization errors that could be introduced.
The state propagation step updates the following $N$ times,

\begin{align}
    \hat{\boldsymbol{x}}^-_{k+1,i} &= \hat{\boldsymbol{x}}_k + f(\hat{\boldsymbol{x}}, \boldsymbol{u}) \frac{T_s}{N}\\
    P_{k+1}^- = A_d(\hat{\boldsymbol{x}}, \boldsymbol{u}) P_k &A_d(\hat{\boldsymbol{x}}, \boldsymbol{u})^\top + (Q + GQ_uG^\top)\big(\frac{T_s}{2}\big)^2,
\end{align}

and the most updated $\hat{\boldsymbol{x}}$ gets used in the calculation of $f$ and $A_d$.
The process noise is denoted as $Q$, the uncertainty on the inputs is given as $Q_u$ and the Jacobian of $f$ with respect to the inputs is given as $G$.

\subsubsection{Measurement Updates}

Similarly to other EKFs, the general measurement update equation used in the ROScopter EKF is factored into Joseph's stabilized form to ensure that the covariance remains positive definite.
It is calculated as,

\begin{align}
    S &= R + CP^-C^\top \\
    K &= P^-C^\top S^{-1} \\
    P^+ &= (I - KC)P(I-KC)^\top + LRL^\top, 
\end{align}
where $S$ denotes the uncertainty on the innovation, $K$ is the Kalman gain and $C$ is the Jacobian of the measurement model with respect to the state.

In general, the covariance $S$ of the innovation $s = z-h$ may have more terms, if $z$ is a function of the states and the measurement.
In which case,

\begin{equation}
    S = FRF^\top + GPG^\top + CP^-C^\top - 2GPC^\top,
\end{equation}
where $F$ is the Jacobian of $z$ with respect to the measurement, $G$ is the Jacobian of $z$ with respect to the estimated state.
As will be seen later, this is useful when the measurement model would otherwise be complex.

Beyond the IMU, the estimator utilizes the following sensors,
- barometer,
- magnetometer,
- and GNSS.

Each measurement model, $h$, and observation Jacobian, $C$, for each sensor are given below.

#### Barometer
The barometer measures the static atmospheric pressure and uses the model,

\begin{equation}
    h_{\text{baro}} = -\rho g p_d
\end{equation}

where $g$ is the acceleration due to gravity, $p_d$ is the estimated down position and $\rho$ is the air density calculated from the absolute altitude above sea level and the 1976 standard atmosphere model for the troposphere.
This yields the expected sensor value for atmospheric pressure.
Taking the derivative with respect to $\boldsymbol{x}$,

\begin{equation}
    C_{\text{baro}} =
    \begin{bmatrix}
        0 & 0 & -\rho g &
        \boldsymbol{0}_{1\times3} &
        \boldsymbol{0}_{1\times3} &
        \boldsymbol{0}_{1\times3} &
    \end{bmatrix}
\end{equation}

#### Magnetometer
The magnetometer measures the intensity of the magnetic field.
This is used to create a measurement of the heading using a tilt-compensated magnetometer model.

\begin{equation}
    z_{\text{mag}} = \text{atan2}(\begin{bmatrix} 0&1&0 \end{bmatrix}R_b^{v_1}\boldsymbol{m},\, \begin{bmatrix} 1&0&0 \end{bmatrix}R_b^{v_1}\boldsymbol{m}) .
\end{equation}
The magnetic field measurements need to be expressed in the $v1$ frame, in other words the inertial frame that is rotated by the yaw of the aircraft.
This allows for the calculation of the angle of the north axis using $\text{atan2}$ which is the heading of the aircraft.

This yields the simple measurement model,

\begin{equation}
    h_{mag} = \psi,
\end{equation}

where $\psi$ is the estimated yaw of the aircraft.

The observation Jacobian is then,

\begin{equation}
    C_{\text{mag}} =
    \begin{bmatrix}
        \boldsymbol{0}_{1\times3} &
        0 & 0 & 1 &
        \boldsymbol{0}_{1\times3} &
        \boldsymbol{0}_{1\times3} &
    \end{bmatrix}.
\end{equation}

However, the contribution of the uncertainty of the states used to calculate $z_{\text{mag}}$ must be appropriately accounted for as in equation for $S$ above.

#### GNSS
The GNSS update includes measurements of the velocity of the aircraft in the inertial frame and the absolute position in latitude and longitude.

FINISH

## Parameters

<!-- TODO: update this -->

| **Parameter** | **Explanation** | **Type** | **Range** |
| :---: | :---: | :---: | :---: |
| `sigma_n_gps` | The standard deviation of gps measurements in the north axis. | double | ~0.01 meters |
| `sigma_e_gps` | The standard deviation of gps measurements in the east axis. | double | ~0.01 meters |
| `sigma_Vg_gps` | The standard deviation of gps ground speed measurements. | double | ~0.005 $\frac{m}{s}$ |
| `sigma_course_gps` | The standard deviation of gps ground course measurements. | double | ~0.00025 radians |
| `sigma_accel` | The standard deviation of accelerometer measurements. | double | ~0.025 $\frac{m}{s^2}$ |
| `sigma_pseudo_wind_n` | The standard deviation of pseudo-measurement along the north axis. | double | ~0.01 |
| `sigma_pseudo_wind_e` | The standard deviation of pseudo-measurement along the east axis. | double | ~0.01 |
| `sigma_heading` | The standard deviation of heading measurement. | double | ~0.01 radians |
| `lpf_a` | The lowpass filter cutoff frequency for the gyro and accelerometers. | double | ~50.0 Hz |
| `lpf_a1` | The lowpass filter cutoff frequency for the pressure sensors. | double | ~8.0 Hz |
| `gps_n_lim` | The limit of GPS measurements in the north direction. | double | $\geq 10000$ meters |
| `gps_e_lim` | The limit of GPS measurements in the east direction. | double | $\geq 10000$ meters |
| `roll_process_noise` | The process noise on the roll propagation. | double | ~0.0001 |
| `pitch_process_noise` | The process noise on the pitch propagation. | double | ~0.0000001 |
| `gyro_process_noise` | The process noise on the gyro inversion. | double |  ~0.13 |
| `pos_process_noise` | The process noise on the position propagation. | double | 0.1 |
| `attitude_initial_cov` | Initial covariance entry for attitude. | double | 5.0 |
| `pos_n_initial_cov` | Initial covariance entry for position north. | double | 0.03 |
| `pos_e_initial_cov` | Initial covariance entry for position east. | double | 0.03 |
| `vg_initial_cov` | Initial covariance entry for groundspeed. | double | 0.01 |
| `chi_initial_cov` | Initial covariance entry for course. | double | 5.0 Degrees |
| `wind_n_initial_cov` | Initial covariance entry for wind north. | double | 0.04 |
| `wind_e_initial_cov` | Initial covariance entry for wind east. | double | 0.04 |
| `psi_initial_cov` | Initial covariance entry for heading. | double | 5.0 |
| `num_propagation_steps` | Number of propagation steps to run between measurements. | int | 10 |
| `max_estimated_phi` | The maximum roll estimate before estimator saturation. | double | ~90.0 degrees |
| `max_estimated_theta` | The maximum pitch estimate before estimator saturation. | double | ~80.0 degrees |
| `estimator_max_buffer` | The buffer that the esimator returns the estimate to after saturation. | double | ~3.0 degrees |
