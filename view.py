import pygame

from constants import BG_COLOR, SCREEN_HEIGHT, SCREEN_WIDTH, TARGET_FPS
from simulation import Simulation, SimulationConfig


class View:
    """Pygame front-end: events, render, fixed frame rate. Logic lives in ``Simulation``."""

    def __init__(self, width, height, simulation=None):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
        self.running = False
        cfg = SimulationConfig(
            world_width=width,
            world_height=height,
            max_steps=0,
        )
        self.sim = simulation or Simulation(cfg)
        self.clock = pygame.time.Clock()

    @property
    def state(self):
        return self.sim.state

    def start(self):
        self.running = True

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.sim.step()

            self.screen.fill(BG_COLOR)
            for a in self.state.actors:
                a.draw(self.screen)

            pygame.display.flip()
            self.clock.tick(TARGET_FPS)
