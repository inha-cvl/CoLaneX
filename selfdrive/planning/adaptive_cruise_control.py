from tabulate import tabulate

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

    def get_target_velocity(self, ego_vel, target_vel):
        
        vel_error = ego_vel - self.object_vel
        safe_distance = ego_vel*self.time_gap
        dist_error = safe_distance-self.object_dist

        acceleration = -(self.vel_gain*vel_error + self.dist_gain*dist_error)
        out_vel = min(ego_vel+acceleration, target_vel)

        data = [
            ["ego_vel", "object_dist", "vel_error", "safe_distance", "dist_error", "acceleration", "out_vel"],
            [ego_vel, self.object_dist, vel_error, safe_distance, dist_error, acceleration, out_vel]
        ]
        # print("Get Target Velocity")
        # print(tabulate(data,  tablefmt="grid"))

        return out_vel
    