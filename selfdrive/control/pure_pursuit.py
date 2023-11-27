import numpy as np

class PurePursuit(object):
    def __init__(self, lfd_gain, wheelbase, min_lfd, max_lfd):
        self.lfd_gain = lfd_gain
        self.wheelbase = wheelbase
        self.min_lfd = min_lfd
        self.max_lfd = max_lfd

    def calculate_steering_angle(self, vehicle_state, path):
        lfd = self.lfd_gain * vehicle_state.velocity
        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

        steering_angle = 0.
        for point in path:
            diff = point - vehicle_state.position
            rotated_diff = diff.rotate(-vehicle_state.heading)
            if rotated_diff.x > 0:
                dis = rotated_diff.distance()
                if dis >= lfd:
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd)
                    break

        return steering_angle