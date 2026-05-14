import numpy as np

# 1. 영상 규격 (리얼센스 표준 해상도)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# 2. 버드아이뷰(BEV) 좌표 설정 (사다리꼴 -> 직사각형 변환)
# [좌하, 우하, 좌상, 우상]
BEV_POINTS = np.float32([
    [-50, 480],   # 좌하
    [690, 480],  # 우하
    [180, 260],  # 좌상
    [420, 260]   # 우상
])

# 3. 조향 제어 파라미터 (PID Gain)
PID_P = 0.5    # 핸들 꺾는 민감도
PID_I = 0.01   
PID_D = 0.1    

# 4. 로봇 구동 사양 
BASE_SPEED = 30      # 기본 주행 속도 (30cm/s)

# [중요!] 팀원 코드의 track_width = 0.50 반영
TRACK_WIDTH = 0.5    # 로봇 좌우 궤도 간격 (0.5m = 50cm)

# [참고] cmd_vel 코드의 sprocket_radius = 0.08 (8cm)
# 비전 코드에서 직접 쓰진 않지만, 정보 보관용으로 메모!
WHEEL_RADIUS = 0.08
