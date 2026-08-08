import random

import pytest

from interception.vector import Vector


class FakeState:
    """
    Minimal stand-in for :class:`~interception.state.State`.

    Actors only need an RNG, world extent, and an obstacle list, so unit
    tests can build one without spawning a whole world.
    """

    def __init__(self, seed=0, width=1500.0, height=1000.0):
        self.rng = random.Random(seed)
        self.width = width
        self.height = height
        self.obstacles = []
        self.targets = []
        self.agents = []


@pytest.fixture
def fake_state():
    return FakeState()


@pytest.fixture
def place():
    """Pin an actor's kinematics, bypassing its randomised spawn."""

    def _place(actor, pos, vel):
        actor.pos = Vector(*pos)
        actor.vel = Vector(*vel)
        actor.prev_pos = actor.pos.copy()
        actor.reorient()
        return actor

    return _place
