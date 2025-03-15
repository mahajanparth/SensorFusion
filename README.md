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
│── kalman_filter/                   # EKF implementation
│   ├── include/kalman_filter/      # EKF headers
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


![Alt System State ](./media/EKFALGO2.png "EKF ALGO")


## State Definition

We define the system state as the vector:

![Alt System State ](./media/8Dof8DofStateVector.png " 8 Dof System State")
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

![Alt Constant Accelration Motion Model ](NonLinearMotionModel.png)

where \(v = \sqrt{v_x^2 + v_y^2}\). 

### Jacobian \(G_t\)
Because \(g\) is nonlinear, the EKF uses the Jacobian:

![Alt Constant Accelration Motion Model ](JacobianOfNonLinearMotionModel.png)

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
ros2 launch kalman_filter_app launch_ekf_node.py

ros2 bag play <bag_file>.bag --clock 

```
---

#Future Scope of the Project :

1) ADD UKF in the kalman core module 
2) compare with constant velocity model 
3) modify the package to take in multiple instances of sensor module 

---
Download ROSBAG LINK : https://www.dropbox.com/scl/fi/tdxin6bzw01siucdv3kgv/linkou-2023-12-27-2-med.zip?rlkey=rcz93bhozjsdymcpn5dqz6rly&e=1&dl=0

---

## Contributors

- **Parth Mahajan** (Author)

## License

This project is licensed under the **MIT License**.



