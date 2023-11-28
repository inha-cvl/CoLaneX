import numpy as np 
import scipy.interpolate

class AdaptiveCruiseControl:
    def __init__(self,  vehicle_length, velocity_gain, distance_gain, time_gap):
        self.vel_gain = velocity_gain #bigger : sensitive with velocity
        self.dist_gain = distance_gain #smaller : sensitive with distance
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_dist= 0
        self.object_vel = 0
    
    def check_objects(self, path):
        goal = path[-1]
        distance_threshold = 0
        for point in path:
            distance_from_path = point.distance(goal)           
            if distance_from_path<=distance_threshold:
                self.object_dist = goal.distance(path[0])-(self.vehicle_length*2)
                self.object_vel = 0
    
    def calculate_curvature(self, path):
        path_array = np.array(path)
        tck, u = scipy.interpolate.splprep([path_array[:, 0], path_array[:, 1]], s=0)
        new_points = scipy.interpolate.splev(np.linspace(0, 1, 1000), tck)
        x, y = new_points[0], new_points[1]
        dx_dt, dy_dt = np.gradient(x), np.gradient(y)
        d2x_dt2, d2y_dt2 = np.gradient(dx_dt), np.gradient(dy_dt)
        curvature = (dx_dt * d2y_dt2 - d2x_dt2 * dy_dt) / ((dx_dt**2 + dy_dt**2)**1.5)        
        avg_curvature = abs(np.mean(curvature))*1000
        co = self.smoothed_deceleration(avg_curvature)
        return avg_curvature, co

    
    def smoothed_deceleration(self, avg_curvature):
        if 5<avg_curvature <= 11:
            co = 0.35
        elif 2<avg_curvature<=5:
            co = 0.2
        elif 0.7<avg_curvature<=2:
            co = 0.1
        else:
            co = 0
        return co

    def get_target_velocity(self, ego_vel, target_vel, co):
        
        vel_error = ego_vel - self.object_vel
        safe_distance = ego_vel*self.time_gap
        dist_error = safe_distance-self.object_dist

        acceleration = -(self.vel_gain*vel_error + self.dist_gain*dist_error)
        target_vel = max(40, target_vel-target_vel*(co))
        out_vel = min(ego_vel+acceleration, target_vel)

        return out_vel
    