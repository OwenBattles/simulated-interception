import random

from vector import Vector
from constants import MAX_X_VEL, MAX_Y_VEL, MAX_X_ACC, MAX_Y_ACC, MAX_XPOS, MAX_YPOS, SCREEN_WIDTH, SCREEN_HEIGHT

class Actor():
    def __init__(self, state_ref):
        # Start positions within screen space
        self.state_ref = state_ref
        self.pos = Vector(random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT))
        self.vel = Vector(random.randint(-MAX_X_VEL, MAX_X_VEL), random.randint(-MAX_Y_VEL, MAX_Y_VEL))
        self.acc = Vector(random.randint(-MAX_X_ACC, MAX_X_ACC), random.randint(-MAX_Y_ACC, MAX_Y_ACC))
        self.mass = 15
        self.max_speed = 5
        self.max_force = 4
        self.orientation = [Vector(1, 0), Vector(0, 1)]

    def move(self):
        pass

    def draw(self, screen):
        pass

    def reorient(self):
        new_forward = self.vel.set_magnitude(1)
        new_side = new_forward.perpendicular()
        self.orientation = [new_forward, new_side]