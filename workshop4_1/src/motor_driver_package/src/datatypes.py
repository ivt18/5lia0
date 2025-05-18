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
    # request types
    MOVEMENT_REQUEST = 1
    TURN_REQUEST = 2

    def __init__(self, request_type, value):
        self.request_type = request_type
        self.value = value
