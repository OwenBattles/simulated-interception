import pygame

from actor import Actor
from constants import MAX_OBSTACLE_RADIUS, MIN_OBSTACLE_RADIUS


class Obstacle(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)
        self.size = self._rng.randint(MIN_OBSTACLE_RADIUS, MAX_OBSTACLE_RADIUS)
        w, h = state_ref.width, state_ref.height
        self.pos.update(
            self._rng.randint(self.size, w - self.size),
            self._rng.randint(self.size, h - self.size),
        )
        self.color = (128, 128, 128)

    def move(self):
        pass

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.pos.x), int(self.pos.y)), self.size)