import numpy as np

A_CRUISE_MAX_VALS = [21, 24, 23, 21]
# A_CRUISE_MAX_VALS = [24, 22, 20, 18]
#A_CRUISE_MAX_BP = [0., 10.0, 25., 50.] #max 50
# A_CRUISE_MAX_BP = [0., 15.0, 30., 60.] #max 60
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.] #max 60


B_CRUISE_MAX_VALS = [20, 23, 25, 30]
A_CRUISE_MIN = 3

class Actuator:
    def __init__(self, acc, steer, v):
        if acc > 0:
            self.accel = self.get_actual_accel(acc*100, v)
            self.brake = 0.
        else:
            self.accel = 0.
            self.brake = self.get_actual_brake(-acc*100, v)
        
        self.steering = steer
    
    def get_actual_accel(self, accel, ego_v):
        _max = np.interp(ego_v, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)
        actual = np.clip(accel, A_CRUISE_MIN, _max)
        return actual

    def get_actual_brake(self, brake, ego_v):
        _max = np.interp(ego_v, A_CRUISE_MAX_BP, B_CRUISE_MAX_VALS)
        actual = np.clip(brake, A_CRUISE_MIN, _max)
        if ego_v < 3:
            actual = _max 
        return actual


    
