import pygame

import agent
from state import State
from constants import SCREEN_WIDTH, SCREEN_HEIGHT, BG_COLOR, AGENT_COLOR, TARGET_COLOR, AGENT_RADIUS, TARGET_RADIUS, TARGET_FPS

class View():
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
        self.running = False
        self.state = State()
        self.clock = pygame.time.Clock()

    def start(self):
        self.running = True

        while self.running:
            # Handle events first
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # Update simulation once per frame
            self.state.update()

            # Efficient draw
            self.screen.fill((255, 255, 255))
            for a in self.state.actors:
                a.draw(self.screen)
  
            pygame.display.flip()
            self.clock.tick(TARGET_FPS)
