class RobotController:
    def __init__(self, config):
        self.cfg = config
        self.error_sum = 0
        self.prev_error = 0

    def calculate_skid_steering(self, target_angle, speed_boost=0):
        # PID 제어로 목표 각속도(omega) 계산
        error = target_angle
        p_term = self.cfg.PID_P * error
        i_term = self.cfg.PID_I * (self.error_sum + error)
        d_term = self.cfg.PID_D * (error - self.prev_error)
        
        omega = p_term + i_term + d_term
        self.prev_error = error
        
        # Skid Steering 공식 적용
        v = self.cfg.BASE_SPEED + speed_boost
        left_v = v - (omega * self.cfg.TRACK_WIDTH / 2)
        right_v = v + (omega * self.cfg.TRACK_WIDTH / 2)
        
        return int(left_v), int(right_v)
