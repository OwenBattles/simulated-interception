import pygame

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
            for target in self.state.targets:
                pygame.draw.circle(self.screen, (255, 0, 0), (int(target.pos.x), int(target.pos.y)), 6)
            for agent in self.state.agents:
                self.draw_oriented_triangle(agent.pos, agent.orientation[0], agent.orientation[1], (0, 0, 255))

            # Flip and cap FPS
            pygame.display.flip()
            self.clock.tick(TARGET_FPS)

    def draw_oriented_triangle(self, position, forward, side, color):
        length = 20 
        width = 10

        tip = position + forward * length
        left_rear = position - forward * length + side * width
        right_rear = position - forward * length - side * width

        pygame.draw.polygon(self.screen, color, [tip.pair(), left_rear.pair(), right_rear.pair() ])
