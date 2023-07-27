from localization.point import Point

class VehicleState:
    def __init__(self, x=0, y=0, heading=0, velocity=0):
        self.position = Point(x,y)
        self.heading = heading
        self.velocity = velocity
