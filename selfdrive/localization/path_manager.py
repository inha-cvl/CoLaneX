import numpy as np

class PathManager:
    def __init__(self, max_velocity):
        self.velocity_profile = max_velocity
        self.local_path_size = 300 #150m
        self.path = []
    
    def update_path(self, path):
        self.path = path
    
    def get_local_path(self, vehicle_state):
        min_distance=float('inf')
        current_waypoint=0
        for i, point in enumerate(self.path):
            dx = point.x - vehicle_state.position.x
            dy = point.y - vehicle_state.position.y
            distance = dx*dx + dy*dy
            if distance < min_distance:
                min_distance = distance
                current_waypoint = i
        if current_waypoint + self.local_path_size < len(self.path):
            local_path = self.path[current_waypoint:current_waypoint + self.local_path_size]
        else:
            local_path = self.path[current_waypoint:]

        return local_path, self.velocity_profile