import random
from constants import MAX_X_VEL, MAX_Y_VEL, MAX_X_ACC, MAX_Y_ACC, MAX_XPOS, MAX_YPOS, SCREEN_WIDTH, SCREEN_HEIGHT

class Actor():
    def __init__(self):
        # Start positions within screen space
        self.x_pos = random.randint(0, SCREEN_WIDTH)
        self.y_pos = random.randint(0, SCREEN_HEIGHT)
        self.x_vel = random.randint(-MAX_X_VEL, MAX_X_VEL)
        self.y_vel = random.randint(-MAX_Y_VEL, MAX_Y_VEL)
        self.x_acc = random.randint(-MAX_X_ACC, MAX_X_ACC)
        self.y_acc = random.randint(-MAX_Y_ACC, MAX_Y_ACC)

    def move(self):
        self.x_vel = max(-MAX_X_VEL, min(MAX_X_VEL, self.x_vel + self.x_acc))
        self.y_vel = max(-MAX_Y_VEL, min(MAX_Y_VEL, self.y_vel + self.y_acc))
        self.x_pos += self.x_vel
        self.y_pos += self.y_vel
        # Keep within screen bounds
        self.x_pos = max(0, min(SCREEN_WIDTH, self.x_pos))
        self.y_pos = max(0, min(SCREEN_HEIGHT, self.y_pos))