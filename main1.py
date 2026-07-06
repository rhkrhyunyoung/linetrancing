import cv2
import numpy as np
from vision_processor import VisionProcessor
from robot_control import RobotController
import config
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()
        
        # 퍼블리셔 및 구독자 설정
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        self.vision = VisionProcessor(config)
        self.controller = RobotController(config)
        
        # 상태 변수
        self.current_yaw_rate = 0.0
        self.prev_left_x = None
        self.prev_right_x = None
        self.last_track_seen_time = time.time()
        
        print("RealSense & ROS 2 연결 성공! 슬립 보정 주행(Topic 모드)을 시작합니다...")

    def imu_callback(self, msg):
        self.current_yaw_rate = msg.angular_velocity.z

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        bev = self.vision.get_bev(frame)
        binary = self.vision.get_binary_track(frame)
        
        left_line, right_line = self.vision.fit_dual_ransac(binary)
        track_width_px = 350
        display_bev = bev.copy()
        y_pts = np.linspace(100, 480, 20).reshape(-1, 1)

        smoothed_left, smoothed_right = None, None

        # 왼쪽 트랙 처리
        curr_left = left_line.predict(y_pts) if left_line else None
        if curr_left is not None:
            if self.prev_left_x is None: self.prev_left_x = curr_left
            smoothed_left = (self.prev_left_x * 0.8) + (curr_left * 0.2)
            self.prev_left_x = smoothed_left
            pts_l = np.int32([np.column_stack([smoothed_left, y_pts])])
            cv2.polylines(display_bev, pts_l, False, (0, 255, 255), 2)
            self.last_track_seen_time = time.time()

        # 오른쪽 트랙 처리
        curr_right = right_line.predict(y_pts) if right_line else None
        if curr_right is None and smoothed_left is not None:
            curr_right = smoothed_left + track_width_px

        if curr_right is not None:
            if self.prev_right_x is None: self.prev_right_x = curr_right
            smoothed_right = (self.prev_right_x * 0.8) + (curr_right * 0.2)
            self.prev_right_x = smoothed_right
            pts_r = np.int32([np.column_stack([smoothed_right, y_pts])])
            cv2.polylines(display_bev, pts_r, False, (255, 0, 0), 2)
            
        twist = Twist()

        # 중앙선 계산 및 제어
        if smoothed_left is not None and smoothed_right is not None:
            smoothed_center = (smoothed_left + smoothed_right) / 2
            pts_c = np.int32([np.column_stack([smoothed_center, y_pts])])
            cv2.polylines(display_bev, pts_c, False, (0, 255, 0), 5)
            
            target_x = smoothed_center[0]
            error = target_x - (config.IMAGE_WIDTH / 2)
            
            l_speed, r_speed = self.controller.calculate_skid_steering(error)
            
            twist.linear.x = float((l_speed + r_speed) / 200.0) 
            target_angular_z = float((r_speed - l_speed) / 200.0)
            
            # IMU 슬립 보정
            slip_correction = (target_angular_z - self.current_yaw_rate) * 0.2
            twist.angular.z = target_angular_z + slip_correction

            # 화면 정보 표시
            cv2.putText(display_bev, f"Error: {error:.1f}", (10, 30), 1, 1.2, (0, 255, 0), 2)
            cv2.putText(display_bev, f"Target Z: {target_angular_z:.2f}", (10, 60), 1, 1.2, (255, 255, 255), 2)
            cv2.putText(display_bev, f"Real Z(IMU): {self.current_yaw_rate:.2f}", (10, 90), 1, 1.2, (255, 255, 0), 2)
        else:
            time_since_lost = time.time() - self.last_track_seen_time
            if time_since_lost < 3.0:
                twist.linear.x = 0.3 
                twist.angular.z = 0.0
                cv2.putText(display_bev, f"Status: Lost (Straight {3.0-time_since_lost:.1f}s)", (10, 120), 1, 1.2, (0, 165, 255), 2)

        self.cmd_pub.publish(twist)
        
        cv2.imshow("BEV Result", display_bev)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
