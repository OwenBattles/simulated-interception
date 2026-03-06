import random

from fleet import Fleet
from target import Target
from obstacle import Obstacle

class State():
    def __init__(self):
        self.obstacles = [Obstacle(self) for _ in range(random.randint(5, 10))] 
        self.targets = [Target(self) for _ in range(random.randint(1, 1))] # change to 1 for now, but can increase later, I may want to create some sort of fleet for targets later
        self.agents = Fleet(1, self).agents
        self.actors = self.agents + self.targets + self.obstacles
    
    def update(self):
        for a in self.actors:
            a.move()

    