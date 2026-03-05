import pygame
from actor import Actor

class Agent(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)
        self.length = 20
        self.width = 10
        self.color = (0, 0, 255)
        self.probe_length = 50

    def move(self):
        target = self.state_ref.targets[0] # TODO: handle multiple targets
        steering_force = (self.get_target_future_pos() - self.pos - self.vel).truncate(self.max_force) 
        self.acc = steering_force / self.mass
        self.vel = (self.vel + self.acc).truncate(self.max_speed)
        self.pos += self.vel
        self.reorient()

    def get_target_future_pos(self):
        target = self.state_ref.targets[0] # TODO: handle multiple targets
        target_future_pos = target.pos + target.vel * (self.pos - target.pos).magnitude() / self.max_speed
        return target_future_pos

    def reorient(self):
        new_forward = self.vel.set_magnitude(1)
        new_side = new_forward.perpendicular()
        self.orientation = [new_forward, new_side]

    def draw(self, screen):
        tip = self.pos + self.orientation[0] * self.length
        left_rear = self.pos - self.orientation[0] * self.length + self.orientation[1] * self.width
        right_rear = self.pos - self.orientation[0] * self.length - self.orientation[1] * self.width

        pygame.draw.polygon(screen, self.color, [tip.pair(), left_rear.pair(), right_rear.pair() ])
