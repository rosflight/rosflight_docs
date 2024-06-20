# Estimator Example

<!-- TODO: rename this to continuous discrete once full state is integrated. -->

## Overview

The `estimator_example` class implements a continuous-discrete Kalman filter as described in section 8.5 of the [UAV book](https://github.com/randybeard/mavsim_public) or 8.6 and 8.7 of volume one of the same book.
Specifically, this estimator uses algorithm 2 of chapter 8 in Volume 1.
It utilizes a two stage estimation process along with low pass filtering the inversion of a few sensor models and direct measurements.
The roll and pitch of the aircraft are estimated first.
This is called the attitude estimation step though not all of the attitude is estimated here.
The other states are then estimated as a all at once.
This is called the position estimation step, though more than just position is estimated during this step.
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

The first step in estimating the attitude is understanding how we propagate our guess between measurements.
Our estimates are on roll angle, $\phi$, and on pitch angle, $\theta$.
These estimates are part of the state $\hat{x}_a$, where the hat over the variable indicates an estimate.

### Propagation

At each call of the estimation algorithm, the estimate from the previous time step is propagated to the next time step.
The propagation step is broken up into `N_` smaller steps to yield an estimate for the current time step.
$N$ is typically 10, meaning that the previous estimate that was updated by a measurement is updated in 10 steps instead of a single step to calculate the estimate at the current time step.
The length of each of the $N$ steps is 1/$N$ the original time step.

The attitude estimation is propagated according to the model $f$:

\begin{equation}
    f =
    \begin{bmatrix}
        \dot{\phi} = p + (q \sin{\phi} + r \cos{\phi}) \tan{\theta} \\
        \dot{\theta} = q \cos{\phi} + r \sin{\phi})
    \end{bmatrix}
\end{equation}

This propagated estimate is then used in the calculation of the Jacobian $A$.

\begin{equation}
    A =
    \begin{bmatrix}
        (q \cos{\phi} - r \sin{\phi})\tan{\theta} & \frac{q\sin{\phi} + r\cos{phi}}{\cos^2{\theta}} \\
        0 & -q \sin{\phi} - r \cos{\phi} \\
    \end{bmatrix}
\end{equation}

This Jacobian is then used to find a second-order approximation of the matrix exponential, $A_d$.

\begin{equation}
    A_d = I + \frac{T_s}{N} A + \frac{1}{2} \frac{T_s^2}{N^2} A^2
\end{equation}

Where $T_s$ is the length of a time step.
$A_d$ is then used to propagate the actual covariance of the estimate.

The process noise due to model uncertainty is defined in the matrix $Q$, but the process uncertainty due to the use of the rate gyro measurements $Q_g$, has not been adjusted for yet.
This is done with the use of the matrix $G$ which takes into account the Coriolis effects of the gyro measurements.
The measurement variance is transformed into process noise due to gyro measurements.
All of these contribute into finding the current covariance of our estimate, $P$.
This is done by the following equation:

$$P_a = A_d P A_d^\top + (Q + G Q_g G^\top) \frac{T_s^2}{N^2}$$

With this propagated estimate and covariance we are now ready for a measurement update.

### Measurement Update

A measurement update provides a check on our propagated estimate and we take this new information and fuse it into our estimate.
The Kalman filter allows us to optimally adjust our estimate, our tuned process noises, and the noise characteristics of our sensor given the measurement.
These noise characteristics are captured in a diagonal matrix, $R_{sensor}$.

Using our estimate and a model set of equations $h$, we predict the measurements the accelerometer will produce.
We will then compare the actual and predicted measurements and optimally adjust our estimate with the new information.
The set of equations, $h$, that predict the 3 measurements of the accelerometer, $y$, is given by:

\begin{equation}
    h = 
    \begin{bmatrix}
        q V_a \sin{\theta} + g \sin{\theta} \\
        r V_a \cos{\theta} - p V_a \sin{\theta} - g \cos{\theta} \sin{\phi} \\
        -q V_a \cos{\theta} - g \cos{\theta} \cos{\phi} \\
    \end{bmatrix}
\end{equation}

This yields a Jacobian $C$:

\begin{equation}
    C = 
    \begin{bmatrix}
        0 & q V_a \cos{\theta} + g \cos{\theta} \\
        -g \cos{\phi}\cos{\theta} & -r V_a \sin{\theta} - p V_a \cos{\theta} + g \sin{\phi} \sin{theta} \\
        g \sin{\phi}\cos{\theta} & (q V_a + g \cos{\phi}) \sin{\theta}
    \end{bmatrix}
\end{equation}

Which is used in finding the Kalman gain $L$.
An intermediate value is calculated called $S^{-1}$.
This value is:

\begin{equation}
    S^{-1} = (R_{accel} + CPC^\top)^{-1}
\end{equation}

This intermediate value is then used to find $L$:

\begin{equation}
    L = PC^\top S^{-1}
\end{equation}

The optimal estimate is then found:

\begin{equation}
    \hat{x}_a^+ =  \hat{x}_a^- + L (y - h)
\end{equation}

Finally, we update the covariance from our new estimate:

\begin{equation}
    P^+ = (I - LC) P^- (I - LC)^\top + LR_{accel}L^\top
\end{equation}

We repeat this cycle until termination of the program.
This estimation scheme split into two parts allows for a clear set of equations and how the estimates affect one another.
We will now move on to the next portion of the estimator that estimates the rest of the states.

## Position Estimation

The position estimation step follows the same algorithm as previous.
Only new values are used for each of the matrices.
Those new entries are shown here, but reference the previous section for details on implementation.

### Propagation


