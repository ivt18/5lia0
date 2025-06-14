class Position:
    def __init__(self, x=0, y=0, angle=0):
        self.x = x
        self.y = y
        self.theta = angle


class SafetyCarPosition:
    def __init__(self, x=0, y=0, d=0, a=0):
        self.x = x
        self.y = y
        self.distance = d
        self.angle = a


class CarConfig:
    def __init__(self, radius, wheelbase):
        self.wheel_radius = radius
        self.wheelbase = wheelbase
