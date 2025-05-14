from dataclasses import dataclass

@dataclass
class CarConfig:
    wheel_radius: float
    wheelbase: float

@dataclass
class Pose:
    x: float
    y: float
    theta: float