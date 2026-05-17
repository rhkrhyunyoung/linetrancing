# Autonomous Line-Tracking Defense Robot
High-Performance Curve Tracking & IMU Slip-Compensated Navigation System based on ROS 2 & OpenCV

This repository features a robust, real-time autonomous navigation system designed for the Defense Robot Competition. The system integrates Adaptive Thresholding, Polynomial Dual RANSAC, and IMU Sensor Fusion to master steep curves and challenging 3D banked tracks under erratic lighting and debris-heavy conditions.

# Tech Stack & Environment
- OS: Ubuntu 22.04 LTS
- Framework: ROS 2 (Humble)
- Language: Python 3
- Libraries: OpenCV, Scikit-learn, NumPy
- Hardware: Intel RealSense D435 (RGB-D), IMU Sensor, Tracked/Skid-Steer Robot Platform

# Key Modules & Architecture
1. Perception & Preprocessing
- Bird’s Eye View (BEV) & Adaptive ROI
- LAB Color Space & Adaptive Thresholding
- Noise Filtering (Morphology & Gaussian Blur)
2. Estimation & Path Planning
- Polynomial Dual RANSAC
- Data Sampling & Real-Time Optimization
- Frame Smoothing (Alpha Blending)
3. Motion Control & Dynamics
- Look-Ahead Distance Target Tracking
- Skid Steering Logic & cmd_vel Publisher
- IMU-Based Real-Time Slip Compensation

# Project Structure
├── config.py             # Pre-configured lane parameters, BEV warp anchors, and resolutions

├── main1.py              # Main ROS 2 executable node housing the control loop & IMU slip logic

├── vision_processor.py   # Computer vision engine (BEV warp, LAB mask, Polynomial RANSAC)

└── robot_control.py      # Skid-steer Kinematics translator computing individual track velocities

# code
```
cd ~/linetrancing
python3 main1.py
```
