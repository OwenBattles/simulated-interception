import math

from vector import Vector

def polar_to_cartesian(r, theta):
        return Vector(r * math.cos(theta), r * math.sin(theta))
    
def cartesian_to_polar(vector):
    r = vector.magnitude()
    theta = math.atan2(vector.y, vector.x)
    return r, theta