import cv2
import numpy as np
import pyrealsense2 as rs
from vision_processor import VisionProcessor
from robot_control import RobotController
import config

import rclpy
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = rclpy.create_node('lane_follower')
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)
    
    vision = VisionProcessor(config)
    controller = RobotController(config)
    
    prev_left_x = None
    prev_right_x = None
    
    print("RealSense & ROS 2 연결 성공! 주행을 시작합니다...")

    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            
            bev = vision.get_bev(frame)
            binary = vision.get_binary_track(frame)
            
            left_line, right_line = vision.fit_dual_ransac(binary)
            display_bev = bev.copy()
            y_pts = np.array([[100], [480]])

            smoothed_left, smoothed_right = None, None

            if left_line:
                curr_left = left_line.predict(y_pts)
                if prev_left_x is None:
                    prev_left_x = curr_left
                smoothed_left = (prev_left_x * 0.8) + (curr_left * 0.2)
                prev_left_x = smoothed_left
                cv2.line(display_bev, (int(smoothed_left[0]), 100), (int(smoothed_left[1]), 480), (0, 255, 255), 2)

            if right_line:
                curr_right = right_line.predict(y_pts)
                if prev_right_x is None:
                    prev_right_x = curr_right
                smoothed_right = (prev_right_x * 0.8) + (curr_right * 0.2)
                prev_right_x = smoothed_right
                cv2.line(display_bev, (int(smoothed_right[0]), 100), (int(smoothed_right[1]), 480), (255, 0, 0), 2)

            if smoothed_left is not None and smoothed_right is not None:
                smoothed_center = (smoothed_left + smoothed_right) / 2
                cv2.line(display_bev, (int(smoothed_center[0]), 100), (int(smoothed_center[1]), 480), (0, 255, 0), 5)
                
                target_x = smoothed_center[1]
                error = target_x - (config.IMAGE_WIDTH / 2)
                
                l_speed, r_speed = controller.calculate_skid_steering(error)
                
                twist = Twist()
                twist.linear.x = float((l_speed + r_speed) / 200.0) 
                twist.angular.z = float((r_speed - l_speed) / 200.0)
                cmd_pub.publish(twist)

                cv2.putText(display_bev, f"Error: {error:.1f}", (10, 30), 1, 1.5, (0, 255, 0), 2)
                cv2.putText(display_bev, f"L_Speed: {int(l_speed)}", (10, 70), 1, 1.5, (255, 255, 255), 2)
                cv2.putText(display_bev, f"R_Speed: {int(r_speed)}", (10, 110), 1, 1.5, (255, 255, 255), 2)

            cv2.imshow("1. Original", frame)
            cv2.imshow("2. Binary", binary)
            cv2.imshow("3. BEV Result", display_bev)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rclpy.spin_once(node, timeout_sec=0)

    finally:
        try:
            cmd_pub.publish(Twist())
        except:
            pass
        pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
