import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor

class VisionProcessor:
    def __init__(self, config):
        self.cfg = config
        src = np.float32(self.cfg.BEV_POINTS) 
        dst = np.float32([[0, 480], [640, 480], [0, 0], [640, 0]])
        self.M = cv2.getPerspectiveTransform(src, dst)

    def get_bev(self, frame):
        return cv2.warpPerspective(frame, self.M, (640, 480))

    def get_binary_track(self, frame):
        bev = self.get_bev(frame)
        lab = cv2.cvtColor(bev, cv2.COLOR_BGR2LAB)
        _, _, b_channel = cv2.split(lab)
        
        binary = cv2.adaptiveThreshold(b_channel, 255, 
                                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                        cv2.THRESH_BINARY_INV, 51, 5)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        binary[:int(480 * 0.20), :] = 0
        return binary

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
                return None

        left_line = get_line(left_mask, 0)
        right_line = get_line(right_mask, mid)
        return left_line, right_line
