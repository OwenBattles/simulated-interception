import pygame

from state import State

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
        TARGET_FPS = 60

        while self.running:
            # Handle events first
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            # Update simulation once per frame
            self.state.update()

            # Efficient draw
            self.screen.fill((255, 255, 255))
            for target in self.state.targets:
                pygame.draw.circle(self.screen, (255, 0, 0), (int(target.x_pos), int(target.y_pos)), 6)
            for agent in self.state.agents:
                pygame.draw.circle(self.screen, (0, 0, 255), (int(agent.x_pos), int(agent.y_pos)), 6)

            # Flip and cap FPS
            pygame.display.flip()
            self.clock.tick(TARGET_FPS)

    def end(self):
        pygame.quit()
