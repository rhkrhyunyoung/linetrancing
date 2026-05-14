import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

class VisionProcessor:
    def __init__(self, config):
        self.cfg = config
        # BEV 매트릭스 계산
        src = np.float32(self.cfg.BEV_POINTS) 
        dst = np.float32([[0, 480], [640, 480], [0, 0], [640, 0]])
        self.M = cv2.getPerspectiveTransform(src, dst)

    def get_bev(self, frame):
        # 이미지를 위에서 내려다보는 시점으로 변환 (BEV)
        return cv2.warpPerspective(frame, self.M, (640, 480))

    def get_binary_track(self, frame):
        bev = self.get_bev(frame)
        lab = cv2.cvtColor(bev, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        
        # 1. 색상 필터링
        _, mask_a = cv2.threshold(a, 115, 255, cv2.THRESH_BINARY_INV)
        _, mask_b = cv2.threshold(b, 150, 255, cv2.THRESH_BINARY_INV)
        binary = cv2.bitwise_and(mask_a, mask_b)
        
        # 2. Morphology OPEN (노이즈 제거)
        kernel_open = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
        
        # 3. Morphology CLOSE (구멍 메우기)
        kernel_close = np.ones((7, 7), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)
        
        # 4. Canny Edge (경계선 추출)
        binary = cv2.Canny(binary, 50, 150)
        
        # 5. [추가] Edge 팽창 (선 두껍게 만들기!)
        kernel_dilate = np.ones((4, 4), np.uint8)
        binary = cv2.dilate(binary, kernel_dilate, iterations=1)
        
        # 6. 최종 ROI 적용
        binary[:int(self.cfg.IMAGE_HEIGHT * 0.25), :] = 0
        
        return binary

    def fit_dual_ransac(self, binary_img):
        height, width = binary_img.shape
        mid = width // 2
        
        left_mask = binary_img[:, :mid]
        right_mask = binary_img[:, mid:]
        
        def get_line(mask, offset_x=0):
            y_coords, x_coords = np.where(mask > 0)
            if len(x_coords) < 50: return None
            
            y_coords = y_coords[::5]
            x_coords = x_coords[::5]
            try:
                ransac = RANSACRegressor(residual_threshold=15, max_trials=50) 
                model = make_pipeline(PolynomialFeatures(degree=2), ransac)
                X = y_coords.reshape(-1, 1)
                y = x_coords + offset_x
                model.fit(X, y)
                return model
            except:
                return None
        return get_line(binary_img[:, :mid], 0), get_line(binary_img[:, mid:], mid)

        left_line = get_line(left_mask, 0)
        right_line = get_line(right_mask, mid)
        
        return left_line, right_line

    def fit_ransac_line(self, binary_img):
        # (기존 main.py와 호환을 위한 함수)
        y_coords, x_coords = np.where(binary_img > 0)
        if len(x_coords) < 50: return None
        try:
            ransac = RANSACRegressor(residual_threshold=10)
            X = y_coords.reshape(-1, 1)
            y = x_coords
            ransac.fit(X, y)
            return ransac
        except:
            return None
