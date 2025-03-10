# Extended Kalman Filter (EKF)

An implementation of an **Extended Kalman Filter** for fusing odometry and imu readings when the system follows a **constant‐acceleration** motion model. This repository demonstrates how to design and implement the EKF step by step, from the state definition to the prediction and update equations. 

---

## Table of Contents

1. [Overview](#overview)  
2. [State Definition](#state-definition)  
3. [Process (Motion) Model](#process-motion-model)  
4. [Control Input](#control-input)  
5. [Prediction Step](#prediction-step)  
6. [Update Step](#update-step)  
7. [Installation and Usage](#installation-and-usage)  
8. [License](#license)  
9. [References](#references)  

---

## Overview

The Extended Kalman Filter (EKF) is a variant of the Kalman Filter used for **nonlinear** systems. It linearizes the nonlinear process and/or measurement functions about the current estimate using **Jacobian** matrices. The steps are:

1. **Predict** the state based on the motion model and control input (if any).  
2. **Update** the state with incoming sensor measurements.

We apply the EKF in an **8‐dimensional** state space to track:
- Position (`x`, `y`)
- Orientation (`theta`)
- Velocity (`v_x`, `v_y`)
- Angular velocity (`omega`)
- Acceleration (`a_x`, `a_y`)

---

## State Definition

We define the system state as the vector:

\[
x_t = 
\begin{bmatrix}
x \\ 
y \\ 
\theta \\
v_x \\
v_y \\
\omega \\
a_x \\
a_y 
\end{bmatrix}
\]

where:

- \(x, y\) : Position in the plane  
- \(\theta\) : Orientation (heading angle)  
- \(v_x, v_y\) : Linear velocities in the \(x\) and \(y\) directions  
- \(\omega\) : Angular velocity (turn rate)  
- \(a_x, a_y\) : Linear accelerations in the \(x\) and \(y\) directions  

The state covariance is an \(8 \times 8\) matrix \(\Sigma\).  

---

## Process (Motion) Model

We assume a **constant acceleration** motion model over a time step \(\Delta_t\). The **nonlinear** function \(g\) predicts the next state \(x_{t}\) from \(x_{t-1}\) and control input \(u_t\):

\[
g(u_t, x_{t-1}) =
\begin{bmatrix}
x + v\cos(\theta)\,\Delta_t + 0.5\,a_x\,\Delta_t^2 \\[6pt]
y + v\sin(\theta)\,\Delta_t + 0.5\,a_y\,\Delta_t^2 \\[6pt]
\theta + \omega\,\Delta_t \\[6pt]
v \cos(\theta) + a_x\,\Delta_t \\[6pt]
v \sin(\theta) + a_y\,\Delta_t \\[6pt]
\omega \\[6pt]
a_x \\[6pt]
a_y
\end{bmatrix}
\]

where \(v = \sqrt{v_x^2 + v_y^2}\). 

### Jacobian \(G_t\)
Because \(g\) is nonlinear, the EKF uses the Jacobian:

\[
G_t = \frac{\partial g}{\partial x}\Big|_{x_{t-1}}
\]

to linearize the process model at the current state estimate.

---

## Control Input

Depending on your application, you can include external inputs (e.g., throttle, steering commands in a vehicle) in \(u_t\). If no external commands exist, you may set \(u_t = \mathbf{0}\) and rely solely on the motion model.

---

## Prediction Step

1. **Predict State Mean**  
   \[
   \bar{\mu}_t = g\bigl(u_t,\mu_{t-1}\bigr)
   \]

2. **Predict State Covariance**  
   \[
   \bar{\Sigma}_t = G_t \,\Sigma_{t-1}\,G_t^T + R_t
   \]
   - \(G_t\) is the Jacobian of \(g\) evaluated at \(\mu_{t-1}\).  
   - \(R_t\) is the process noise covariance.

---

## Update Step

Upon receiving a measurement \(z_t\), the EKF incorporates the sensor reading:

1. **Measurement Prediction**:  
   \[
   \hat{z}_t = h(\bar{\mu}_t)
   \]  
   where \(h\) is the measurement function (could be linear or nonlinear).

2. **Jacobian \(H_t\)**:  
   \[
   H_t = \frac{\partial h}{\partial x}\Big|_{\bar{\mu}_t}
   \]

3. **Kalman Gain**:  
   \[
   K_t = \bar{\Sigma}_t\,H_t^T \Bigl(H_t\,\bar{\Sigma}_t\,H_t^T + Q_t\Bigr)^{-1}
   \]

4. **Update Mean**:  
   \[
   \mu_t = \bar{\mu}_t + K_t\bigl(z_t - \hat{z}_t\bigr)
   \]

5. **Update Covariance**:  
   \[
   \Sigma_t = \bigl(I - K_t\,H_t\bigr)\,\bar{\Sigma}_t
   \]

- \(Q_t\) is the measurement noise covariance.  
- \(z_t\) is the actual measurement vector at time \(t\).

---

