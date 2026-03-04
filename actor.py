import random

class Actor():
    def __init__(self):
        self.x_pos = random.randint(0, 800)
        self.y_pos = random.randint(0, 600)
        self.x_vel = random.randint(-5, 5)
        self.y_vel = random.randint(-5, 5)
        self.x_acc = random.randint(-1, 1)
        self.y_acc = random.randint(-1, 1)

    def move(self):
        pass