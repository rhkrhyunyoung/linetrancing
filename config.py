import numpy as np

# 영상 처리 관련
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
# BEV (Bird's Eye View) 변환을 위한 4개 지점 (대회장 바닥에서 직접 찍어야 함)
BEV_POINTS = np.float32([[150, 450], [490, 450], [50, 200], [590, 200]]) 

# 조향 및 제어 관련
PID_P = 0.5       #핸들 꺾는정도
PID_I = 0.01
PID_D = 0.1
BASE_SPEED = 30  # 로봇 기본 속도
TRACK_WIDTH = 0.4 # 로봇 좌우 궤도 간격 (m)

# config.py 에 추가 (화면 보면서 튜닝해야 함!)
# [좌하, 우하, 좌상, 우상] 순서의 사다리꼴 좌표
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
BEV_POINTS = [[50, 480], [590, 480], [250, 150], [390, 150]]
