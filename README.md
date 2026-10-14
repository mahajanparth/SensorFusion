# Sensor Fusion: EKF for Wheel Odometry and IMU

A ROS2 / C++ Extended Kalman Filter that fuses wheel odometry and IMU data for a ground robot, using a constant-acceleration motion model. The project covers the filter design (state, prediction and update), three variants of the filter, and an evaluation on a real outdoor dataset with GPS as ground truth.

![EKF variants in RViz2](eval/results/rviz/ekf_variants_rviz.gif)

*RViz2 playback of the Linkou dataset at 4x speed. The three filter variants run in parallel and are aligned to GPS (white dots). EKF-C (yellow) follows the GPS track and its 3σ ellipses grow as position is dead-reckoned. EKF-A (orange) and EKF-B (green) drift together with the raw wheel odometry (gray) while reporting less than 1 m of uncertainty.*

### Highlights

- 8-state EKF (position, heading, velocity, yaw rate, acceleration) with pluggable motion and observation models
- Three filter variants compared on a 240 s, 247 m outdoor drive
- Best variant cuts position error from 19.6 m (wheel odometry) to 1.7 m and heading error from 36° to 1.2°
- RViz2 view with live paths and 3σ covariance ellipses, runnable through Docker (`eval/rviz_variants.sh`)

---

## Table of Contents

1. [Overview](#overview)
2. [State Definition](#state-definition)
3. [Process (Motion) Model](#process-motion-model)
4. [Control Input](#control-input)
5. [Prediction Step](#prediction-step)
6. [Update Step](#update-step)
7. [Installation and Usage](#installation-and-usage)
8. [Results on the Linkou Dataset](#results-on-the-linkou-dataset)
9. [Future Scope](#future-scope)
10. [License](#license)

---

## Overview

The Extended Kalman Filter (EKF) is a variant of the Kalman Filter used for **nonlinear** systems. It linearizes the nonlinear process and/or measurement functions about the current estimate using **Jacobian** matrices. The steps are:

1. **Predict** the state based on the motion model and control input (if any).  
2. **Update** the state with incoming sensor measurements.

We apply the EKF in an **8-dimensional** state space to track:
- Position (`x`, `y`)
- Orientation (`theta`)
- Velocity (`v_x`, `v_y`)
- Angular velocity (`omega`)
- Acceleration (`a_x`, `a_y`)

## Features

- Implements **nonlinear motion models** such as constant acceleration.
- Supports **sensor observation models**, including IMU and odometry fusion.
- Modular **EKF framework** with customizable process and observation models.
- **ROS2-compatible** for real-time robotic state estimation.
- **Configurable process noise and sensor noise covariance matrices.**


```
src/
│── motion_model/                   # Motion model implementations
│   ├── include/motion_model/       # Motion model headers
│   ├── src/                        # Source files for motion models
│   ├── CMakeLists.txt              # CMake configuration for motion models
│   ├── package.xml                 # ROS2 package definition
│
│── observation_model/               # Observation model implementations
│   ├── include/observation_model/  # Observation model headers
│   ├── src/                        # Source files for observation models
│   ├── CMakeLists.txt              # CMake configuration for observation models
│   ├── package.xml                 # ROS2 package definition
│
│── kalman_filter_core/              # EKF implementation
│   ├── include/kalman_filter_core/ # EKF headers
│   ├── src/                        # Source files for EKF
│   ├── CMakeLists.txt              # CMake configuration for EKF
│   ├── package.xml                 # ROS2 package definition
│
│── kalman_filter_app/               # Main application
│   ├── src/                        # Main application source files
│   ├── CMakeLists.txt              # CMake configuration for application
│   ├── package.xml                 # ROS2 package definition
│
│── common_utils/                    # Utility functions (e.g., coordinate conversion)
│   ├── include/common_utils/       # Utility headers
│   ├── src/                        # Source files for utilities
│   ├── CMakeLists.txt              # CMake configuration for utilities
│
```

---


![Alt System State ](./media/EKF_ALGO2.png "EKF ALGO")


## State Definition

We define the system state as the vector:

![Alt System State ](./media/8DofStateVector.png " 8 Dof System State")
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

![Alt Constant Accelration Motion Model ](./media/NonLinearMotionModel.png)

where \(v = \sqrt{v_x^2 + v_y^2}\). 

### Jacobian \(G_t\)
Because \(g\) is nonlinear, the EKF uses the Jacobian:

![Alt Constant Accelration Motion Model ](./media/JacobianOfNonLinearMotionModel.png)

to linearize the process model at the current state estimate.

---

## Control Input

No Control Input was considered in this application 

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

Upon receiving a measurement (z_t), the EKF incorporates the sensor reading:

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

- (Q_t) is the measurement noise covariance.  
- (z_t) is the actual measurement vector at time (t).




## Installation and Usage


## Dependencies

To use this project, ensure the following dependencies are installed:

- **ROS2 (Humble/Foxy recommended)**
- **Eigen3** for matrix operations
- **C++17 or higher**
- **CMake** (minimum version 3.5)
- **Colcon** (for ROS2 package builds)

## Installation & Build

### **1. Clone the Repository**

```bash

git clone https://github.com/mahajanparth/SensorFusion --recursive
cd SensorFusion

```

### **2. Install Dependencies**

```bash
sudo apt update && sudo apt install -y ros-foxy-eigen3 ros-foxy-colcon
```

### **3. Build the Project**

```bash
colcon build --symlink-install
```

### **4. Source the Workspace**

```bash
source install/setup.bash
```

## Run Procedure

### **1. Launch the Full Sensor Fusion Pipeline**

```bash
# variant:=A|B|C (default C), inputs default to /wheel_odom and /imu/data_raw
ros2 launch kalman_filter_app launch_ekf_node.py variant:=C

ros2 bag play <bag_dir> --topics /wheel_odom /imu/data_raw
```

The fused estimate is published on `/ekf/odom` (`nav_msgs/Odometry`). To reproduce the results below, see [eval/README.md](eval/README.md).

---

## Results on the Linkou Dataset

The filter was tested on the Linkou bag (download link below), a 240 s, 247 m outdoor drive recorded with wheel odometry (`/wheel_odom`) and an IMU (`/imu/data_raw`).

- Ground truth is the GPS `/fix` topic. It is a standard fix, not RTK, with σ between 0.3 and 5 m, so errors below about 1.5 m are within GPS noise.
- Every trajectory is aligned to GPS with an SE(2) Umeyama fit (rotation and translation, no scale) before computing the error.
- Heading error is measured against the `/odom` reference recorded in the bag.

### Filter variants

All three variants share the EKF core and the 8-state vector `[x, y, θ, vx, vy, ω, ax, ay]`. They differ in the motion model and in which measurements they fuse.

| | EKF-A | EKF-B | EKF-C |
|---|---|---|---|
| Motion model | speed √(vx²+vy²) along θ | world-frame constant acceleration | world-frame constant acceleration |
| Jacobian evaluated at | predicted mean | previous posterior | previous posterior |
| Measurements | odom x, y, θ; IMU θ, ω, ax, ay | odom x, y, θ; IMU θ, ω; odom v_fwd, v_lat; IMU ax, ay | IMU θ, ω; odom v_fwd, v_lat; IMU ax, ay |
| Fuses odom pose | yes | yes | no |
| IMU acceleration | raw (gravity tilt included) | gravity removed, body frame | gravity removed, body frame |
| Measurement noise R | 0.05 for all | tuned per sensor | tuned per sensor |

Select a variant with `variant:=A`, `B` or `C` at launch. C is the default.

### Summary

| Method | Position error vs GPS (ATE) | Heading RMSE | Drift per 10 m (RPE) |
|---|---:|---:|---:|
| Raw wheel odometry | 19.58 m | 36.1° | 0.60 m |
| EKF-A | 19.57 m | 18.0° | 4.49 m |
| EKF-B | 19.30 m | 3.5° | 10.19 m |
| **EKF-C** | **1.72 m** | **1.2°** | **0.27 m** |

Results for four 60 s segments are in [eval/results/metrics.md](eval/results/metrics.md). EKF-C has the lowest position error and drift in every segment.

![All variants, full run](eval/results/traj_full.png)

On this dataset the wheel odometry heading drifts by about 130° over the run (its yaw rate reads roughly 20% high), while the IMU heading stays within 2° of the reference. How each variant handles that disagreement explains most of the difference in results.

### EKF-A: original model

![EKF-A](eval/results/variant_A.png)

**Model.** The simplest complete IMU and odometry EKF. Every sensor goes into one measurement vector and the observation model is linear (H is a selection matrix), which makes it easy to follow and debug. The constant-acceleration model handles smooth speed changes better than a constant-velocity model.

**Result.** EKF-A lies almost on top of the wheel odometry (19.57 m vs 19.58 m).
- The odometry x and y get the same weight as every other measurement (R = 0.05), so the position follows the drifting wheel path.
- The heading ends up between the wheel and IMU headings. That halves the heading error (18° vs 36°) without removing it.
- Raw accelerometer readings, with gravity leaking in through tilt, are compared directly against world-frame accelerations. This adds jitter and raises the drift per 10 m.
- The 3σ band stays under 1 m while the real error is around 20 m, so the covariance is overconfident.

### EKF-B: corrected model with odom pose

![EKF-B](eval/results/variant_B.png)

**Model.**
- World-frame constant acceleration keeps the sign of the velocity, so reversing can be represented, and the Jacobian is exact.
- The Jacobian is evaluated at the previous estimate, as in the standard EKF.
- Gravity is removed from the accelerometer using the IMU orientation. Accelerations and wheel velocities are compared in the body frame, where those sensors measure.
- The odom pose is taken relative to the starting pose, so it shares a reference with the IMU yaw. This replaces the hardcoded -90° rotation.

**Result.**
- Heading improves to 3.5° RMSE and no longer drifts (right panel).
- Position does not improve. The odom x and y were integrated with the drifting wheel heading, so they are not independent information, and fusing them keeps pulling the estimate back onto the wheel path (19.30 m).
- The good heading and the biased position conflict, which causes jumps (10.2 m of drift per 10 m travelled) and a 3σ band that is again too small.

### EKF-C: corrected model, wheel velocity and IMU only

![EKF-C](eval/results/variant_C.png)

**Model.** Same motion model as B, but it fuses only what each sensor measures well:
- wheels: forward speed, plus lateral speed close to zero for a non-holonomic robot
- IMU: heading, yaw rate and gravity-compensated acceleration

Position is never measured directly. It comes from integrating the wheel speed along the IMU-corrected heading, which treats wheel odometry as a velocity sensor rather than a pose sensor.

**Result.**
- The trajectory stays on the GPS track for the whole drive (1.72 m vs 19.58 m for wheel odometry). Most of the remaining error is in the first 30 s, where the GPS fix is weakest (open circles).
- Heading error stays flat around 0° (1.2° RMSE).
- The 3σ band grows steadily as position is dead-reckoned and always contains the actual error, so the reported covariance is reliable.

![Segments](eval/results/traj_segments.png)

---

## Future Scope

1. Add a UKF to the kalman core module
2. Compare against a constant-velocity motion model
3. Support multiple instances of each sensor

---
**Dataset (Linkou rosbag):** https://www.dropbox.com/scl/fi/tdxin6bzw01siucdv3kgv/linkou-2023-12-27-2-med.zip?rlkey=rcz93bhozjsdymcpn5dqz6rly&e=1&dl=0

---

## Contributors

- **Parth Mahajan** (Author)

## License

This project is licensed under the **MIT License**.



