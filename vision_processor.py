import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor

class VisionProcessor:
    def __init__(self, config):
        self.cfg = config
        # BEV 매트릭스 계산
        src = np.float32(self.cfg.BEV_POINTS) 
        dst = np.float32([[0, 480], [640, 480], [0, 0], [640, 0]])
        self.M = cv2.getPerspectiveTransform(src, dst)

    def get_bev(self, frame):
        return cv2.warpPerspective(frame, self.M, (640, 480))

    def get_binary_track(self, frame):
        bev = self.get_bev(frame)
        lab = cv2.cvtColor(bev, cv2.COLOR_BGR2LAB)
        _, _, b_channel = cv2.split(lab)
        
        # Adaptive Threshold (덩어리 잡기용)
        binary = cv2.adaptiveThreshold(b_channel, 255, 
                                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                        cv2.THRESH_BINARY_INV, 51, 5)
        
        # 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # ROI (위 20% 지우기)
        binary[:int(480 * 0.20), :] = 0
        return binary

    # 기존 main.py와 호환되도록 이름 유지
    def fit_ransac_line(self, binary_img):
        y_coords, x_coords = np.where(binary_img > 0)
        if len(x_coords) < 50:
            return None
        try:
            ransac = RANSACRegressor(residual_threshold=10)
            X = y_coords.reshape(-1, 1)
            y = x_coords
            ransac.fit(X, y)
            return ransac
        except:
            return None

    # 좌우 개별 인식용 (나중에 쓸 것)
    def fit_dual_ransac(self, binary_img):
        height, width = binary_img.shape
        mid = width // 2
        left_mask = binary_img[:, :mid]
        right_mask = binary_img[:, mid:]
        
        def get_line(mask, offset_x=0):
            y_coords, x_coords = np.where(mask > 0)
            if len(x_coords) < 30:
                return None
            try:
                ransac = RANSACRegressor(residual_threshold=10)
                X = y_coords.reshape(-1, 1)
                y = x_coords + offset_x
                ransac.fit(X, y)
                return ransac
            except:
                # 에러 포인트: 여기 밑에 반드시 return None이 있어야 함!
                return None

        left_line = get_line(left_mask, 0)
        right_line = get_line(right_mask, mid)
        return left_line, right_line
