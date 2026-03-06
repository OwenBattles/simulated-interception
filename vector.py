import math

from constants import MAX_XPOS, MAX_YPOS, SCREEN_WIDTH, SCREEN_HEIGHT

class Vector():
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar)
    
    def update(self, x=None, y=None):
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
    
    def pair(self):
        return (self.x, self.y)

    def magnitude(self):
        return (self.x**2 + self.y**2)**0.5

    def truncate(self, max_value):
        mag = self.magnitude()
        if mag > max_value:
            return self * (max_value / mag)
        return self

    def set_magnitude(self, magnitude):
        current_magnitude = (self.x**2 + self.y**2)**0.5
        if current_magnitude == 0:
            return Vector(0, 0)
        scale = magnitude / current_magnitude
        return self * scale
    
    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector(0, 0)
        return self / mag
    
    def perpendicular(self):
        return Vector(-self.y, self.x)
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def dist_to(self, other):
        return (self - other).magnitude()
    
    def rotate(self, angle):
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        return Vector(
            self.x * cos_theta - self.y * sin_theta,
            self.x * sin_theta + self.y * cos_theta
        )
    
    