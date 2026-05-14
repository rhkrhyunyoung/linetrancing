import cv2
import numpy as np
import pyrealsense2 as rs
from vision_processor import VisionProcessor
from robot_control import RobotController
import config

# ROS 2 라이브러리 및 메시지 추가
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu # IMU 메시지 추가

# IMU 데이터를 전역적으로 저장할 변수
current_yaw_rate = 0.0

def imu_callback(msg):
    global current_yaw_rate
    # IMU의 실제 회전 속도(z축 각속도)를 업데이트
    current_yaw_rate = msg.angular_velocity.z

def main():
    global current_yaw_rate
    
    # 1. ROS 2 초기화 (사용자님 코드 그대로 유지)
    rclpy.init()
    node = rclpy.create_node('lane_follower')
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # [추가] IMU 구독자 생성
    imu_sub = node.create_subscription(Imu, '/imu/data', imu_callback, 10)

    # 2. 리얼센스 설정
    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)
    
    vision = VisionProcessor(config)
    controller = RobotController(config)
    
    # Smoothing 변수
    prev_left_x = None
    prev_right_x = None
    
    print("RealSense & ROS 2 연결 성공! 슬립 보정 주행을 시작합니다...")

    try:
        while rclpy.ok():
            # 3. 프레임 획득
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            
            # 4. 영상 처리 (이미 vision_processor에서 Canny Edge가 적용됨!)
            bev = vision.get_bev(frame)
            binary = vision.get_binary_track(frame)
            
            # 5. 좌우 라인 개별 인식
            left_line, right_line = vision.fit_dual_ransac(binary)
            track_width_px = 350
            display_bev = bev.copy()
            y_pts = np.linspace(100, 480, 20).reshape(-1, 1)

            smoothed_left, smoothed_right = None, None

            # 6. 왼쪽 트랙 처리 (노란색 + Smoothing)
            curr_left = left_line.predict(y_pts) if left_line else None
            if curr_left is not None:
                if prev_left_x is None: prev_left_x = curr_left
                smoothed_left = (prev_left_x * 0.8) + (curr_left * 0.2)
                prev_left_x = smoothed_left
                # 곡선 그리기
                pts_l = np.int32([np.column_stack([smoothed_left, y_pts])])
                cv2.polylines(display_bev, pts_l, False, (0, 255, 255), 2)
            # 7. 오른쪽 트랙 처리 (파란색 + Smoothing)
            curr_right = right_line.predict(y_pts) if right_line else None
            # [추가] 오른쪽이 없으면 왼쪽 기반으로 추정 (가상선 생성)
            if curr_right is None and smoothed_left is not None:
                curr_right = smoothed_left + track_width_px

            if curr_right is not None:
                if prev_right_x is None: prev_right_x = curr_right
                smoothed_right = (prev_right_x * 0.8) + (curr_right * 0.2)
                prev_right_x = smoothed_right
                # 곡선 그리기
                pts_r = np.int32([np.column_stack([smoothed_right, y_pts])])
                cv2.polylines(display_bev, pts_r, False, (255, 0, 0), 2)

            # 8. 중앙선 계산 및 [모터 제어 / ROS 2 발행]
            if smoothed_left is not None and smoothed_right is not None:
                smoothed_center = (smoothed_left + smoothed_right) / 2
                pts_c = np.int32([np.column_stack([smoothed_center, y_pts])])
                cv2.polylines(display_bev, pts_c, False, (0, 255, 0), 5)
                
                target_x = smoothed_center[0]
                error = target_x - (config.IMAGE_WIDTH / 2)
                
                # 모터 속도 계산 (Skid Steering)
                l_speed, r_speed = controller.calculate_skid_steering(error)
                
                # [ROS 2 cmd_vel 발행 및 슬립 보정]
                twist = Twist()
                twist.linear.x = float((l_speed + r_speed) / 200.0) 
                
                # --- [핵심: IMU 슬립 보정 로직] ---
                target_angular_z = float((r_speed - l_speed) / 200.0)
                
                # 슬립 보정: (명령한 회전 속도 - 실제 IMU 회전 속도) 만큼 더해줌
                # 0.1은 보정 게인입니다. 로봇이 휘청거리면 이 값을 조절하세요!
                slip_correction = (target_angular_z - current_yaw_rate) * 0.2
                twist.angular.z = target_angular_z + slip_correction
                # -------------------------------
                
                cmd_pub.publish(twist)

                # 화면에 정보 표시 (IMU 값도 추가)
                cv2.putText(display_bev, f"Error: {error:.1f}", (10, 30), 1, 1.2, (0, 255, 0), 2)
                cv2.putText(display_bev, f"Target Z: {target_angular_z:.2f}", (10, 60), 1, 1.2, (255, 255, 255), 2)
                cv2.putText(display_bev, f"Real Z(IMU): {current_yaw_rate:.2f}", (10, 90), 1, 1.2, (255, 255, 0), 2)

            # 9. 화면 표시
            cv2.imshow("1. Original", frame)
            cv2.imshow("2. Binary ", binary) # 창 제목 변경
            cv2.imshow("3. BEV Result", display_bev)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # ROS 2 이벤트 처리
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
