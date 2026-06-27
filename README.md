# Robot Line-Tracing with Curvature Perception & Slip Compensation
Advanced Autonomous Driving System based on ROS 2 & RealSense
This project implements a high-performance line-tracing algorithm that overcomes the limitations of traditional point-to-point tracking. By leveraging Polynomial Curve Fitting and IMU-based slip compensation, it ensures stable high-speed cornering even on complex, high-curvature tracks.

# Tech Stack & Environment
- Framework: ROS 2 (Humble/Foxy)
- Language: Python 3.10+
- Vision: OpenCV, Intel RealSense SDK (pyrealsense2)
- Math: NumPy (for Polynomial Regression & Matrix operations)
- Hardware: Intel RealSense D435, Skid-Steer Mobile Robot, IMU Sensor

# Key Modules & Architecture
1. High-Order Polynomial Path Fitting
Unlike conventional line-tracers that connect discrete points with straight lines, this system uses Polynomial Fitting to model the track as a continuous mathematical function.
-Benefit: Provides a smooth trajectory even when the line is partially obscured or highly curved.
-Robustness: Effectively filters out visual noise and maintains a stable "center-line" prediction.

2. Target Tracking via Look-Ahead Distance
Inspired by the Pure Pursuit algorithm, the system calculates a dynamic target point based on a defined Look-Ahead Distance.
-Predictive Control: Instead of reacting to the immediate error under the robot, it looks ahead to anticipate upcoming curves.
-Stability: This approach significantly reduces oscillations during high-speed transitions between straights and curves.

3. Skid-Steering Logic & geometry_msgs/Twist Publisher
Designed for skid-steer mobile platforms, the controller translates path-following logic into precise motion commands.
-ROS 2 Integration: Publishes real-time velocity commands to the /cmd_vel topic.
-Kinematics: Optimized steering-to-velocity mapping to ensure smooth rotation and forward momentum balance.

4. Real-time Slip Compensation using IMU Data
To bridge the gap between simulation and reality, an IMU-based feedback loop is integrated.
-Slip Detection: Compares the commanded angular velocity with the actual yaw rate from the IMU.
-Active Compensation: Automatically adjusts the wheel power to counteract drifting or mechanical slip on low-friction surfaces, ensuring the robot stays on its intended trajectory.


# Project Structure
├── config.py             # Pre-configured lane parameters, BEV warp anchors, and resolutions

├── main1.py              # Main ROS 2 executable node housing the control loop & IMU slip logic

├── vision_processor.py   # Computer vision engine (BEV warp, LAB mask, Polynomial RANSAC)

└── robot_control.py      # Skid-steer Kinematics translator computing individual track velocities

# Getting Started
Installation
```
cd ~/linetrancing
pip3 install numpy opencv-python pyrealsense2
python3 main1.py
```

# graph TD
    A[Intel RealSense D435] -->|RGB Frame| B(OpenCV Image Processing)
    B -->|Binary/Filtered| C(2nd Order Polynomial Fitting)
    C -->|Curve Equation| D(Look-Ahead Target Calculation)
    D -->|Target Error| E(Skid-Steering Controller)
    F[IMU Sensor] -->|Real-time Yaw Rate| G(Slip Compensation Logic)
    G --> E
    E -->|Twist Msg| H[cmd_vel Publisher]
    H --> I[Mobile Robot Platform]

# Control Logic Overview
1. Perception: Capture RGB-D frames -> Thresholding -> Lane pixel extraction.

2. Modeling: Fit y=ax2+bx+c to the lane pixels.
3. Targeting: Find the (x,y) coordinate at the Look-Ahead distance.
4. Correction: Calculate steering angle -> Apply IMU Slip Compensation -> Publish to cmd_vel.

# 📐 Mathematical Approach
To achieve smooth cornering, the system fits a 2nd-degree polynomial:
$$f(x) = ax^2 + bx + c$$
The **Look-Ahead distance ($L_d$)** is then used to find the target steering point, significantly reducing oscillations compared to pure error-based control.
