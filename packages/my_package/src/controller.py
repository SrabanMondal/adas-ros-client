import numpy as np

class PurePursuitController:
    def __init__(self, wheel_base=0.1, v=0.2):
        self.wheel_base = wheel_base  # Duckiebot wheelbase ~0.1m
        self.v = v                    # forward velocity

    def compute_control(self, car_center, lookahead_point):
        cx, cy = car_center
        lx, ly = lookahead_point

        dx = lx - cx
        dy = cy - ly  # because image y axis is inverted
        alpha = np.arctan2(dy, dx)

        Ld = np.sqrt(dx**2 + dy**2)
        if Ld < 1e-6:
            return 0.0, 0.0

        delta = np.arctan2(2*self.wheel_base*np.sin(alpha), Ld)
        omega = delta

        vel_left  = self.v - (omega * self.wheel_base / 2.0)
        vel_right = self.v + (omega * self.wheel_base / 2.0)
        return vel_left, vel_right
