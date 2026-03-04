import random

from agent import Agent
from target import Target

class State():
    def __init__(self):
        self.targets = [Target() for _ in range(random.randint(1, 5))]
        self.agents = [Agent() for _ in range(random.randint(1, 5))]
    
    def update(self):
        """Update all actors' positions once per frame."""
        for a in self.agents:
            # Agents implement their own move method (may be a no-op)
            if hasattr(a, "move"):
                a.move()
        for t in self.targets:
            if hasattr(t, "move"):
                t.move()
    