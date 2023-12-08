import numpy as np
from config.config import Config
from localization.path_manager import PathManager
from control.actuator import Actuator
from control.pid import PID
from control.pure_pursuit import PurePursuit
from localization.point import Point
from planning.adaptive_cruise_control import AdaptiveCruiseControl

class SelfDrive:
    def __init__(self, map):
        config = Config(map)
        self.pm = PathManager(config['planning']['velocity_profile']['max_velocity'])        
        self.acc = AdaptiveCruiseControl(config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control'], max_velocity=config['planning']['velocity_profile']['max_velocity'], **config['map']['base_lla'])
        self.pid = PID(sampling_time = 1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(wheelbase=config['common']['wheelbase'], steer_ratio=config['common']['steer_ratio'],steer_max=config['common']['steer_max'], **config['control']['pure_pursuit'])
        self.p_steer_cmd = 0
        self.base_lla = config['map']['base_lla']
        self.start_point = Point()

    def execute(self, mode, vehicle_state, path, lidar_object):
        if mode == 0:
            return Actuator(-100, vehicle_state.heading, vehicle_state.velocity), [], 0, []
        
        if np.all(path[0] != self.start_point):
            self.pm.update_path(path)
            self.start_point = path[0]

        local_path, planned_velocity = self.pm.get_local_path(vehicle_state)
        self.acc.check_objects(local_path)
        lidar_bsd = self.acc.check_bsd(local_path, vehicle_state, lidar_object)
        _, co = self.acc.calculate_curvature(local_path)
        target_velocity = self.acc.get_target_velocity(vehicle_state.velocity, planned_velocity, co)
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        steer_cmd = self.pure_pursuit.calculate_steering_angle(vehicle_state, local_path)
        steer_cmd = self.pure_pursuit.saturate_steering_angle(steer_cmd, self.p_steer_cmd)
        self.p_steer_cmd = steer_cmd

        return Actuator(acc_cmd, steer_cmd, vehicle_state.velocity), local_path, target_velocity, lidar_bsd
        
def main():
    self_drive = SelfDrive()
    self_drive.execute()

if __name__ == '__main__':
    main()