# QBot Trajectory Tracking using LQR Controller

This document outlines the real-time trajectory tracking strategy for a differential drive QBot using a Linear Quadratic Regulator (LQR) controller, as implemented and verified on hardware using MATLAB/Simulink.

## 1. System Description

The QBot Platform is a Differential Drive Wheeled Mobile Robot (DDWMR) designed for research and educational purposes. It serves as a robust testbed for developing and validating control algorithms.

<img src="https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/bot_image.png?raw=true" alt="QBot Image" style="width: 200px; float: right; margin-left: 20px;">

### 1.1. Physical Parameters

- **Mass of QBot (including batteries):** 2.2 kg  
- **Moment of Inertia (I):** 0.0757 kg·m²  
- **Radius of each wheel (R):** 0.0345 m  
- **Distance between wheels (d):** 0.192 m  



### 1.2. Hardware Components
* **Onboard Computational Unit:** Raspberry Pi 4 (4GB RAM) for real-time processing and control.
* **Environmental Perception:**
    * Leishen LiDAR M10P for high-resolution mapping.
    * Intel RealSense D435 RGB-D camera for stereo vision and depth sensing.
    * Grayscale global-shutter CSI camera for line-following tasks.
* **Actuators:** Dual DC motors, independently controlled for differential drive motion.
* **Communication:** Wi-Fi for wireless control, data streaming, and remote monitoring.
* **Electronics Stack:** Custom electronics including motor drivers, Inertial Measurement Unit (IMU), and wheel encoders.
* **Software Integration:** Supports MATLAB/Simulink via Quanser's QUARC real-time control software.

## 2. Bot Kinematics

The QBot's kinematics are described by the standard unicycle model, representing the robot as a single point in a 2D plane defined by its pose.

<img src="https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/schematic.png?raw=true" alt="QBot Image" style="width: 200px; float: right; margin-left: 20px;">

### 2.1. Robot Pose
The pose of the robot in a global coordinate frame is defined by the vector $q=[x,y,\theta]^{T}$, where $(x,y)$ are the Cartesian coordinates of the robot's center point and $\theta$ is its orientation angle with respect to the X-axis.

### 2.2. Wheel Velocities
The linear velocities of the right $(v_{R})$ and left $(v_{L})$ wheels are determined by their respective angular velocities $(\omega_{R},\omega_{L})$ and the wheel radius, R:
* $v_{R}=R\omega_{R}$
* $v_{L}=R\omega_{L}$

### 2.3. Linear and Angular Velocities of the Robot
The robot's overall linear velocity, $v_{cc}$, and angular velocity, $\omega_{cc}$, can be calculated from the individual wheel velocities:
* $v_{cc}=\frac{v_{R}+v_{L}}{2}$
* $\omega_{cc}=\frac{v_{L}-v_{R}}{d}$

### 2.4. Non-holonomic Constraint
A fundamental characteristic of a DDWMR is its non-holonomic constraint, which prevents it from moving sideways. This means the robot's velocity component perpendicular to its orientation must be zero:
* $\dot{x}\sin\theta-\dot{y}\cos\theta=0$
### 2.5. Complete Kinematic Model

The complete kinematic model of the QBot Platform, which describes the evolution of its pose over time, is given by the following set of equations:

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/Screenshot%202025-07-17%20133125.png?raw=true" alt="QBot Image" style="width: 100%; float: right; margin-left: 20px;">

This model is used for linearization and the design of the LQR controller.

## 3. LQR Setup

The Linear Quadratic Regulator (LQR) is used to design an optimal control law that balances state regulation against control effort.

### 3.1. State and Control Vectors
$$
X = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \quad
u = \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

### 3.2. Cost Function
The cost function to be minimized is defined as:
$$J=\int_{0}^{T}(X^{T}QX+u^{T}Ru)dt+X^{T}(T)FX(T)$$
Where:
* **Q Matrix:** $Q=diag(8,8,10)$ - penalizes position and heading error.
* **R Matrix:** $R=diag(10,10)$ - penalizes control effort.
* **F Matrix:** $F=Q$ - terminal state cost.

### 3.3. Linearized Model

The system is linearized around the reference trajectory at each time step. The linearized model is:

$$
\dot{X} = AX + Bu
$$

Where:

**A Matrix:**

$$
A = \begin{bmatrix}
0 & 0 & -v\sin\theta \\
0 & 0 & v\cos\theta \\
0 & 0 & 0
\end{bmatrix}
$$

**B Matrix:**

$$
B = \begin{bmatrix}
\cos\theta & 0 \\
\sin\theta & 0 \\
0 & 1
\end{bmatrix}
$$


### 3.4. Reference Trajectories

#### Path 1: Circular Trajectory
* $x_{ref}(t)=R\sin(\omega t)$
* $y_{ref}(t)=R(1-\cos(\omega t))$
* $\theta_{ref}(t)=\tan^{-1}(\frac{dy_{ref}}{dt}/\frac{dx_{ref}}{dt})$

  <img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/track_0.4.png?raw=true" alt="QBot Image" style="width: 50%; float: right; margin-left: 20px;">

Nominal control inputs for the circular trajectory are:
* $v_{nom}=R\omega$
* $\omega_{nom}=\omega$
* u_nom = [ v_nom ; ω_nom ]ᵀ

#### Path 2: Smooth Cosine Trajectory
The reference trajectory is defined as:
* $x_{ref}(t)=0.1t$
* $y_{ref}(t)=0.1t+\cos(0.1\pi t)-1$
* $\theta_{ref}(t)=\tan^{-1}(\frac{dy_{ref}}{dt}/\frac{dx_{ref}}{dt})$

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/cosine_path.png?raw=true" alt="QBot Image" style="width: 50%; float: right; margin-left: 20px;">
Derivatives for nominal velocities are taken as:
* $\frac{dx_{ref}}{dt}=0.1$
* $\frac{dy_{ref}}{dt}=0.1-0.1\pi\sin(0.1\pi t)$

Hence, the nominal linear and angular velocities are:
- $v_{nom} = R\omega$
- $\omega_{nom} = \omega$

Nominal control vector:

$$
u_{nom} = \begin{bmatrix}
v_{nom} \\
\omega_{nom}
\end{bmatrix}
$$

### 3.5. Control Input Calculation
At each step, the gain K is retrieved based on the current time and applied to compute the control input:
$$u=u_{nom}-K(X-X_{ref})$$

## 4. Simulink Setup

The LQR is implemented inside a MATLAB function block in Simulink. The Riccati equation is solved using `ode45` over a backward time horizon.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/lqr_setup.jpg?raw=true" alt="QBot Image" style="width: 100%; float: right; margin-left: 20px;">

### 4.1. Architecture Overview
1. **Input Processing:** The model begins with the acquisition of wheel speed data (in radians per second) from the encoders, which measure the angular velocities of the left and right wheels. These inputs are fed into the Inverse Kinematics (IK) block.
2. **Inverse Kinematics (IK) Block:** The IK block processes the wheel speed data to compute the robot's state vector, consisting of the Cartesian coordinates $(x, y)$ and the orientation angle $(\theta)$. This block transforms the individual wheel velocities into the robot's linear velocity $(v)$ and angular velocity $(\omega)$, accounting for the non-holonomic constraints of the DDWMR.
3. **Controller Block:** The computed state vector is passed to the LQR controller, which is implemented as a MATLAB function block. The LQR controller solves the Riccati equation iteratively to determine the optimal feedback gain matrix. This gain is applied to minimize a cost function that balances state deviation and control effort, producing the optimal control inputs $(v, \omega)$.
4. **Reference Trajectory Generation:** The model includes blocks for generating a reference circular trajectory $(x_{ref}, y_{ref}, \theta_{ref})$ and nominal velocities $(v_{nom}, \omega_{nom})$. These reference values are compared with the current state to compute the error, which the LQR controller uses to adjust the control inputs.
5. **Output Processing:** The optimal $v$ and $\omega$ values are discretized and filtered through low-pass filters to ensure smooth control signals. These outputs are then directed to the hardware interface, which drives the dual DC motors via the QBot alpha driver. The model also outputs additional sensor data, such as wheel positions, motor commands, accelerometer readings, gyroscope data, motor currents, and battery voltage, for monitoring and validation.
6. **Feedback Loop:** The system incorporates a feedback loop where the encoder readings and sensor data are continuously fed back to refine the state estimation and control inputs, ensuring robust trajectory tracking performance.

## 5. Results

### For Circular Trajectory

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/lqr_cir.gif?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

### For Cosine Trajectory

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/lqr_cos.gif?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

The LQR controller's performance was validated by tracking two different trajectories: a circular path and a smooth cosine path. For each, both simulation and hardware implementation results are included.

**Note on Initial Overshoot:** The robot remains stationary for the first 2 seconds. During this delay, the reference point advances, resulting in a large initial error and a corresponding overshoot in both $v$ and $\omega$. However, after this transient phase, the controller successfully brings the bot onto the trajectory.

### 5.1. Case 1: Circular Path

#### Simulation Results:
* **Trajectory Tracking:** Figure 6(a) demonstrates near-perfect trajectory tracking. The measured trajectory (blue) aligns very closely with the reference circular path (red dashed), indicating excellent tracking accuracy.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/trajectory_0.4_sim.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

* **Velocity and Angular velocity Response:** Figures 7 & 8 present the linear velocity ($v$) and angular velocity ($\omega$) over time. Both signals remain stable and consistent after the initial overshoot.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/linear_velocity_0.4.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/omega_0.4.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

#### Experimental Results:
* **Trajectory Tracking:** Figure 6(b) shows the experimental $x-y$ position vs reference. The measured trajectory on hardware closely follows the reference path.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/tracking_0.4.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

* **Velocity and Angular velocity Response:** Figures 8, 10, and 11 show the real linear and angular velocity responses for different radii. The velocity responses show stable behavior after the initial overshoot.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/v_0.4_white.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/w_0.4_white.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

* **Control Effort:** Figure 12 shows the controller effort for 0.45m and 0.5m radii. The control efforts are observed to be within reasonable limits.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/Controller effort_0.4.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">
<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/controller effort_0.45.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">
<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/control effort_0.5.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

### 5.2. Case 2: Arbitrary Path (Smooth Cosine Trajectory)

#### Simulation Results:
* **Trajectory Tracking Comparison:** Figure 13 compares simulated (left) and experimental (right) trajectory tracking. In both cases, the robot's measured path (blue) closely follows the reference path (red dashed), validating the control strategy. Minor deviations in the experimental plot are due to real-world factors such as sensor noise and actuation delays.

<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/xy_cos.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">
<img src= "https://github.com/Zeista01/LQR-control-on-DDWMR/blob/main/Results/xy_plot_lqr_exp_arb.png?raw=true" alt="QBot Image" style="width: 30%; float: right; margin-left: 20px;">

* **Velocity and Angular velocity Response:** Figure 14 presents the linear velocity ($v$) and angular velocity ($\omega$) over time. Both signals remain smooth and consistent, without significant overshoot or oscillations, reflecting the controller's ability to maintain stable motion.

#### Experimental Results:
* **Trajectory Tracking:** The hardware implementation successfully tracks the smooth cosine path.
* **Velocity and Angular velocity Response:** Figure 15(a) and 15(b) show the real linear and angular velocity responses.
* **Control Effort:** Figure 15(c) and 15(d) show the controller efforts for 0.45m and 0.5m radii. The control efforts remain within expected limits.
