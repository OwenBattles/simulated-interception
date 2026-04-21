import random

from constants import SCREEN_HEIGHT, SCREEN_WIDTH
from fleet import Fleet
from obstacle import Obstacle
from target import Target


class State:
    """Shared world: dimensions, RNG, and all actors. Pygame-free."""

    def __init__(
        self,
        seed=None,
        width=None,
        height=None,
    ):
        self.width = width if width is not None else SCREEN_WIDTH
        self.height = height if height is not None else SCREEN_HEIGHT
        self.rng = random.Random(seed) if seed is not None else random.Random()
        self.obstacles = []
        self.targets = []
        self.agents = []
        self.actors = []
        self._build_world()

    def _build_world(self):
        n_obs = self.rng.randint(5, 10)
        self.obstacles = [Obstacle(self) for _ in range(n_obs)]
        self.targets = [Target(self) for _ in range(self.rng.randint(1, 1))]
        self.agents = Fleet(1, self).agents
        self.actors = self.agents + self.targets + self.obstacles

    def reset(self, seed):
        """Re-seed and rebuild all entities (same dimensions)."""
        self.rng.seed(seed)
        self._build_world()

    def update(self):
        for a in self.actors:
            a.move()
