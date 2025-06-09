class CarConfig:
    def __init__(self, radius, wheelbase):
        self.wheel_radius = radius
        self.wheelbase = wheelbase


class Position:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.theta = angle


class MovementRequest:
    def __init__(self, car, safety_car):
        self.car_pos = car
        self.safety_car_pos = safety_car


class SafetyCarPosition:
    def __init__(self, x=0, y=0, d=0, a=0):
        self.x = x
        self.y = y
        self.distance = d
        self.angle = a

