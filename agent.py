import pygame
from actor import Actor

class Agent(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)

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

    def draw(self, screen):
        length = 20 
        width = 10
        color = (0, 0, 255)

        tip = self.pos + self.orientation[0] * length
        left_rear = self.pos - self.orientation[0] * length + self.orientation[1] * width
        right_rear = self.pos - self.orientation[0] * length - self.orientation[1] * width

        pygame.draw.polygon(screen, color, [tip.pair(), left_rear.pair(), right_rear.pair() ])
