import numpy as np

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = 10

class Actuator:
    def __init__(self, acc, steering, v):
        ego_v = v*3.6
        if acc > 0:
            self.accel = self.get_actual_accel(acc, ego_v)
            self.brake = 0.
        else:
            self.accel = 0.
            self.brake = self.get_actual_brake(-acc)
        self.steering = steering
    
    def get_actual_accel(self, accel, ego_v):
        gain = 20
        _max = np.interp(ego_v, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)
        actual = np.clip(accel, A_CRUISE_MIN, _max*gain)
        return actual

    def get_actual_brake(self, brake):
        _max = 60
        _min = 10
        return np.clip(brake, _min, _max)
    
