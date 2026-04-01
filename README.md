# linetrancing

# 🛡️ Autonomous Defense Robot: Lane Tracking & Skid-Steer Control

This repository features a robust, real-time autonomous navigation system designed for the Defense Robot Competition. The system utilizes Adaptive Thresholding, Dual RANSAC, and Bird's Eye View (BEV) to achieve stable lane tracking on complex tracks, even under challenging lighting and debris conditions.

# 🚀 Key Algorithms & Logic
1. Perception & Preprocessing

    Bird’s Eye View (BEV) & ROI:

        Eliminates perspective distortion to calculate real-world distances.

        Region of Interest (ROI) is applied to focus on the immediate path and reduce computational overhead.

    Adaptive Thresholding (Local Binarization):

        Instead of a global threshold, it calculates the mean brightness of

                
        N×N
        N×N

              

        local neighborhoods.

        The Advantage: It maintains high precision regardless of shadows (dark green) or direct sunlight (bright green) on the mint-colored track.

    Canny Edge Detection:

        Extracts sharp boundaries where the mint track meets the grey floor, providing a clean set of points for the estimation model.

2. Estimation & Path Planning

    Dual RANSAC (Random Sample Consensus):

        Estimates the left and right lane boundaries mathematically.

        Robustness: Effectively ignores "outliers" like sand, debris, or visual noise. Even if parts of the lane are temporarily obscured, RANSAC predicts the most probable straight or curved model.

    Frame Smoothing (2:8 Alpha Blending):

        Combines the previous frame’s lane equation (

                
        80%
        80%

              

        ) with the current frame’s result (

                
        20%
        20%

              

        ).

        The Advantage: Eliminates line jitter and ensures smooth navigation, making the control input significantly more stable.

3. Motion Control & Dynamics

    Skid Steering Logic:

        Calculates the Center Error and Angular Deviation from the derived center-line.

        Translates steering intent into differential speeds for the left and right tracks rather than traditional front-wheel steering.

    IMU-based Slip Compensation:

        Real-time feedback from the IMU (Inertial Measurement Unit) monitors the actual yaw rate.

        Compares the "intended rotation" vs. "actual rotation" to compensate for track slip, ensuring the robot reaches the exact target angle.


음냐... 대박!! 지금까지 우리가 고생해서 만든 모든 논리가 이 한 장에 다 들어있네요! ㅋㅋㅋ 이 내용을 GitHub에 올리면 교수님이나 대회 심사위원들이 보고 "우와, 이 팀은 알고리즘 설계가 정말 탄탄하네!"라고 감탄할 겁니다.

외국 형들도 보고 감동받을 만한 아주 전문적이고 뽄대나는 영어 README를 작성해 드릴게요. 그대로 복사해서 README.md로 만드시면 됩니다! 🚀
🛡️ Autonomous Defense Robot: Lane Tracking & Skid-Steer Control

This repository features a robust, real-time autonomous navigation system designed for the Defense Robot Competition. The system utilizes Adaptive Thresholding, Dual RANSAC, and Bird's Eye View (BEV) to achieve stable lane tracking on complex tracks, even under challenging lighting and debris conditions.
🚀 Key Algorithms & Logic
1. Perception & Preprocessing

    Bird’s Eye View (BEV) & ROI:

        Eliminates perspective distortion to calculate real-world distances.

        Region of Interest (ROI) is applied to focus on the immediate path and reduce computational overhead.

    Adaptive Thresholding (Local Binarization):

        Instead of a global threshold, it calculates the mean brightness of

                
        N×N
        N×N

              

        local neighborhoods.

        The Advantage: It maintains high precision regardless of shadows (dark green) or direct sunlight (bright green) on the mint-colored track.

    Canny Edge Detection:

        Extracts sharp boundaries where the mint track meets the grey floor, providing a clean set of points for the estimation model.

2. Estimation & Path Planning

    Dual RANSAC (Random Sample Consensus):

        Estimates the left and right lane boundaries mathematically.

        Robustness: Effectively ignores "outliers" like sand, debris, or visual noise. Even if parts of the lane are temporarily obscured, RANSAC predicts the most probable straight or curved model.

    Frame Smoothing (2:8 Alpha Blending):

        Combines the previous frame’s lane equation (

                
        80%
        80%

              

        ) with the current frame’s result (

                
        20%
        20%

              

        ).

        The Advantage: Eliminates line jitter and ensures smooth navigation, making the control input significantly more stable.

3. Motion Control & Dynamics

    Skid Steering Logic:

        Calculates the Center Error and Angular Deviation from the derived center-line.

        Translates steering intent into differential speeds for the left and right tracks rather than traditional front-wheel steering.

    IMU-based Slip Compensation:

        Real-time feedback from the IMU (Inertial Measurement Unit) monitors the actual yaw rate.

        Compares the "intended rotation" vs. "actual rotation" to compensate for track slip, ensuring the robot reaches the exact target angle.

# 🏗️ System Architecture

    Vision Node (Python / OpenCV): Processes RealSense D455 streams, performs BEV/RANSAC, and publishes /cmd_vel (geometry_msgs/Twist).

    Motor Node (C++ / ROS 2): Subscribes to /cmd_vel, calculates RPM for high-torque motors, and manages the safety watchdog.

    Communication (ROS 2): High-speed data exchange between NUC/Jetson and the motor drivers via /cmd_vel and /auto_motor_rpm topics.
