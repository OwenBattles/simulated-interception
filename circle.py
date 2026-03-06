import pygame

from actor import Actor

class Circle(Actor):
    def __init__(self, state_ref, agent):
        super().__init__(state_ref)
        self.agent = agent
        self.radius = agent.length * 1.5    
        self.pos = agent.pos + agent.forward_vec * 100

    def move(self):
        self.radius = self.agent.length * 1.5
        self.pos = self.agent.pos + self.agent.forward_vec * 100

    def draw(self, screen):
        pygame.draw.circle(screen, (0, 255, 0), (int(self.pos.x), int(self.pos.y)), self.radius)

    def intersects_obstacle(self, obstacle):
        distance = (self.pos - obstacle.pos).magnitude()
        return distance < self.radius + obstacle.size
    