import cv2
import numpy as np
import pyrealsense2 as rs
from vision_processor import VisionProcessor
from robot_control import RobotController
import config

# ROS 2 라이브러리
import rclpy
from geometry_msgs.msg import Twist

def main():
    # 1. ROS 2 초기화
    rclpy.init()
    node = rclpy.create_node('lane_follower')
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # 2. 리얼센스 설정
    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)
    
    vision = VisionProcessor(config)
    controller = RobotController(config)
    
    # Smoothing 변수 (과거의 선 위치 기억용)
    prev_left_x = None
    prev_right_x = None
    
    print("RealSense & ROS 2 연결 성공! 주행을 시작합니다...")

    try:
        while rclpy.ok():
            # 3. 프레임 획득
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            
            # 4. 영상 처리 (BEV 및 이진화)
            bev = vision.get_bev(frame)
            binary = vision.get_binary_track(frame)
            
            # 5. 좌우 라인 개별 인식
            left_line, right_line = vision.fit_dual_ransac(binary)
            display_bev = bev.copy()
            y_pts = np.array([[100], [480]])

            smoothed_left, smoothed_right = None, None

            # 6. 왼쪽 트랙 처리 (노란색 + Smoothing)
            if left_line:
                curr_left = left_line.predict(y_pts)
                if prev_left_x is None:
                    prev_left_x = curr_left
                smoothed_left = (prev_left_x * 0.8) + (curr_left * 0.2)
                prev_left_x = smoothed_left
                cv2.line(display_bev, (int(smoothed_left[0]), 100), (int(smoothed_left[1]), 480), (0, 255, 255), 2)

            # 7. 오른쪽 트랙 처리 (파란색 + Smoothing)
            if right_line:
                curr_right = right_line.predict(y_pts)
                if prev_right_x is None:
                    prev_right_x = curr_right
                smoothed_right = (prev_right_x * 0.8) + (curr_right * 0.2)
                prev_right_x = smoothed_right
                cv2.line(display_bev, (int(smoothed_right[0]), 100), (int(smoothed_right[1]), 480), (255, 0, 0), 2)

            # 8. 중앙선 계산 및 [모터 제어 / ROS 2 발행]
            if smoothed_left is not None and smoothed_right is not None:
                # 좌우 선의 평균으로 중앙선(초록색) 도출
                smoothed_center = (smoothed_left + smoothed_right) / 2
                cv2.line(display_bev, (int(smoothed_center[0]), 100), (int(smoothed_center[1]), 480), (0, 255, 0), 5)
                
                # 조향 오차 계산
                target_x = smoothed_center[1]
                error = target_x - (config.IMAGE_WIDTH / 2)
                
                # 모터 속도 계산 (Skid Steering)
                l_speed, r_speed = controller.calculate_skid_steering(error)
                
                # [ROS 2 cmd_vel 발행]
                twist = Twist()
                # 선속도(Linear)와 각속도(Angular)로 변환 (스케일은 로봇에 맞춰 조절)
                twist.linear.x = float((l_speed + r_speed) / 200.0) 
                twist.angular.z = float((r_speed - l_speed) / 200.0)
                cmd_pub.publish(twist)

                # 화면에 정보 표시
                cv2.putText(display_bev, f"Error: {error:.1f}", (10, 30), 1, 1.5, (0, 255, 0), 2)
                cv2.putText(display_bev, f"L_Speed: {int(l_speed)}", (10, 70), 1, 1.5, (255, 255, 255), 2)
                cv2.putText(display_bev, f"R_Speed: {int(r_speed)}", (10, 110), 1, 1.5, (255, 255, 255), 2)

            # 9. 화면 표시
            cv2.imshow("1. Original", frame)
            cv2.imshow("2. Binary", binary)
            cv2.imshow("3. BEV Result", display_bev)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # ROS 2 이벤트 처리
            rclpy.spin_once(node, timeout_sec=0)

    finally:
        # 종료 시 안전하게 멈춤
        try:
            cmd_pub.publish(Twist())
        except:
            pass
        pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
