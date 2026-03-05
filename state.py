import random

from agent import Agent
from target import Target

class State():
    def __init__(self):
        self.targets = [Target() for _ in range(random.randint(1, 1))] # change to 1 for now, but can increase later
        self.agents = [Agent() for _ in range(random.randint(2, 2))]
    
    def update(self):
        """Update all actors' positions once per frame."""
        for a in self.agents:
            # Agents implement their own move method (may be a no-op)
            if hasattr(a, "move"):
                a.move(self.targets[0]) # TODO: handle multiple targets
        for t in self.targets:
            if hasattr(t, "move"):
                t.move(self.agents[0]) # TODO: handle multiple agents
    