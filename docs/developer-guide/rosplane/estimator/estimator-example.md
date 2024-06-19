# Estimator Example

<!-- TODO: rename this to continuous discrete once full state is integrated. -->

## Overview

The `estimator_example` class implements a continuous-discrete Kalman filter as described in section 8.5 of the [UAV book](https://github.com/randybeard/mavsim_public) or 8.6 and 8.7 of volume one of the same book.
It utilizes a two stage estimation process along with low pass filtering the inversion of a few sensor models and direct measures.
The roll and pitch of the aircraft are estimated first, this is called the attitude estimation step though not all of the attitude is estimated here.
The other states are then estimated as a whole, this is called the position estimation step, though more than position is estimated during this step.
The estimator runs on a set timer with a configurable frequency (see Parameters section for details).

## Nomenclature

| Symbol | Meaning | Range |
|:------:|:-------:| :---: |
|$\large{\chi}$| Course/Heading | $[-\pi,\pi)$ |
|$\large{\phi}$| Roll | $[-\pi,\pi)$ |
|$\large{\theta}$| Theta | $[-\pi,\pi)$ |
|$\large{\psi}$| Yaw | $[-\pi,\pi)$ |
|$\large{h}$| Altitude | - |
|$\large{p}$| Roll Rate | - |
|$\large{q}$| Pitch Rate | - |
|$\large{r}$| Yaw Rate | - |
|$\large{V_a}$| Airspeed | $\geq 0$ |

## Sensor Model Inversion

The roll, pitch and yaw rates are directly measured by the rate gyro and low pass filtered.
The low pass filter is a simple alpha filter described by:

$$
    a_{n} = a_{n-1} \alpha + (1 - \alpha) a_{measured}
$$

Where $a$ is the state.
This filter technique is used on the measurements before they are used in an estimate throughout the estimator.

The following are calculated directly from a model of the sensor:

<center>

| State | Measured Value | Model Equation | Inversion Equation |
|:--:|:--:|:--:|:--:|
| $h$ - altitude | $P$ - absolute pressure (Pa) | $P = \rho_{air} gh$ | $h = \frac{\rho_{air} g}{P}$|
| $V_a$ - airspeed | $\Delta P$ - differential pressure (Pa) | $\Delta P = \frac{1}{2} \rho_{air} V_a^2$ | $V_a = \sqrt{\frac{2}{\rho_{air} g} \Delta P}$|

</center>

These values are then used in the first step of the estimator.

## Attitude Estimation

### Propagation

At each call of the estimation algorithm, the estimate from the previous time step is propagated to the next time step.
It is propagated a number of times, `N_`, to yield and estimate for the current time step.
`N_` is typically 10, meaning that the previous estimate that was updated by a measurement is updated in 10 steps to get what the estimate at the current time step.
The attitude estimation is propagated according to the equations:

$$
    \dot{\phi} = p + (q \sin{\phi} + r \cos{\phi}) \tan{\theta} \\
    \dot{\theta} = q \cos{\phi} + r \sin{\phi})
$$

This is called `f_a_` in the code.

This propagated estimate is then used in the calculation of the Jacobian.
The Jacobian matrix, `A_a_` is:

\begin{bmatrix}
    (q \cos{\phi} - r \sin{\phi})\tan{\theta} & \frac{q\sin{\phi} + r\cos{phi}}{\cos^2{\theta}} \\
    0 & -q \sin{\phi} - r \cos{\phi} \\
\end{bmatrix}

This Jacobian is then used to find a second-order approximation of the matrix exponential, `A_d_`.
`A_d_` is then used to propagate the actual covariance of the estimate.

The process noise due to model uncertainty is defined in the matrix `Q_a_`, but the process uncertainty due to the use of the rate gyro measures has not been adjusted for yet.
This is done with the use of the matrix `G` which takes into account the Coriolis effects of the measures.
The measurement variance is transformed into process noise due to gyro measures.
All of these contribute into finding the current covariance of our estimate, `P_a_`.
This is done by the following equation:

`P_a_ = A_d_ * P_a_ * A_d_.transpose() + (Q_a_ + G * Q_g_ * G.transpose()) * pow(Ts/N_,2)`

Where `Ts` is the time step between measurement updates.
With this propagated estimate and covariance we are now ready for a measurement update.

### Measurement Update


