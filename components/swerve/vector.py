from dataclasses import dataclass

import math

@dataclass
class Polar:
    magnitude: float
    theta: float

    def to_cartesian(self) -> "Cartesian":
        r, theta = self.magnitude, self.theta
        theta *= math.pi/180
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return Cartesian(x, y)
    
    def to_polar(self) -> "Polar":
        return self

    def __add__(self, vec: "Polar") -> "Polar":
        return self.to_cartesian() + vec.to_cartesian()

@dataclass
class Cartesian:
    x: float
    y: float

    def to_polar(self) -> Polar:
        x, y = self.x, self.y
        r = math.sqrt(x**2 + y**2)
        if x == 0:
            if y > 0:
                return Polar(y, 90)
            elif y < 0:
                return Polar(-y, 270)
            else:
                return Polar(0, 0)
        else:
            theta = math.atan(y/x) * 180/math.pi
        if x < 0:
            theta += 180
        elif y < 0:
            theta += 360
        return Polar(r, theta)
    
    def to_cartesian(self) -> "Cartesian":
        return self

    def __add__(self, vec: "Cartesian") -> "Cartesian":
        vec = vec.to_cartesian()
        return Cartesian(self.x + vec.x, self.y + vec.y)
