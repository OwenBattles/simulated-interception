import pygame
import random

from actor import Actor
from constants import MAX_OBSTACLE_RADIUS, MIN_OBSTACLE_RADIUS, SCREEN_HEIGHT, SCREEN_WIDTH

class Obstacle(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)
        self.size = random.randint(MIN_OBSTACLE_RADIUS, MAX_OBSTACLE_RADIUS)
        self.pos.update(random.randint(self.size, SCREEN_WIDTH - self.size), random.randint(self.size, SCREEN_HEIGHT - self.size))
        self.color = (128, 128, 128)

    def move(self):
        pass

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.pos.x), int(self.pos.y)), self.size)