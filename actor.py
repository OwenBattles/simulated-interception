import random

from vector import Vector
from constants import MAX_X_VEL, MAX_Y_VEL, MAX_X_ACC, MAX_Y_ACC, MAX_XPOS, MAX_YPOS, SCREEN_WIDTH, SCREEN_HEIGHT

class Actor():
    def __init__(self):
        # Start positions within screen space
        self.pos = Vector(random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT))
        self.vel = Vector(random.randint(-MAX_X_VEL, MAX_X_VEL), random.randint(-MAX_Y_VEL, MAX_Y_VEL))
        self.acc = Vector(random.randint(-MAX_X_ACC, MAX_X_ACC), random.randint(-MAX_Y_ACC, MAX_Y_ACC))

    def move(self):
        pass