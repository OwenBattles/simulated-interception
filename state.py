import random

from fleet import Fleet
from target import Target

class State():
    def __init__(self):
        self.targets = [Target() for _ in range(random.randint(1, 1))] # change to 1 for now, but can increase later
        self.agents = Fleet(1).agents
    
    def update(self):
        """Update all actors' positions once per frame."""
        for a in self.agents:
            # Agents implement their own move method (may be a no-op)
            if hasattr(a, "move"):
                a.move(self.targets[0]) # TODO: handle multiple targets
        for t in self.targets:
            if hasattr(t, "move"):
                t.move(self.agents[0]) # TODO: handle multiple agents
    