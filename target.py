from actor import Actor
import pygame

class Target(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)
        self.max_speed = 3.5 # Example max speed, can be adjusted

    def move(self):
        # TODO: implement more advanced move logic
        self.pos += self.vel

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), 6)
