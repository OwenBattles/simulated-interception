import pygame

from actor import Actor

class Circle(Actor):
    def __init__(self, agent, state_ref, probe_distance):
        super().__init__(state_ref)
        self.agent = agent
        self.radius = agent.length * 1.5    
        self.pos = agent.pos + agent.forward_vec * probe_distance
        self.color = (0, 255, 0)

    def move(self):
        self.pos = self.agent.pos + self.agent.forward_vec * self.agent.probe_distance

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.pos.x), int(self.pos.y)), self.radius)

    def intersects_obstacle(self, obstacle):
        distance = (self.pos - obstacle.pos).magnitude()
        return distance < self.radius + obstacle.size
    