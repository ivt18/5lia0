class CarConfig:
    def __init__(self, radius, wheelbase):
        self.wheel_radius = radius
        self.wheelbase = wheelbase


class Pose:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.theta = angle

