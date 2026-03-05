from actor import Actor
from vector import Vector

class Agent(Actor):
    def __init__(self):
        super().__init__()
        self.max_speed = 4 # Example max speed, can be adjusted

    def move(self, target):
        # TODO: Implement movement logic for the agent
        self.vel = (target.pos - self.pos) - self.vel  # Simple seek behavior
        if self.vel.magnitude() > self.max_speed:
            self.vel = self.vel.set_magnitude(self.max_speed)
        self.pos += self.vel
