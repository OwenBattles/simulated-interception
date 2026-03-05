from actor import Actor
from vector import Vector

class Agent(Actor):
    def __init__(self):
        super().__init__()

    def move(self, target):
        # TODO: Implement movement logic for the agent
        steering_force = (target.pos - self.pos - self.vel).truncate(self.max_force) 
        self.acc = steering_force / self.mass
        self.vel = (self.vel + self.acc).truncate(self.max_speed)
        self.pos += self.vel
        self.reorient()

    def reorient(self):
        new_forward = self.vel.set_magnitude(1)
        new_side = new_forward.perpendicular()
        self.orientation = [new_forward, new_side]
