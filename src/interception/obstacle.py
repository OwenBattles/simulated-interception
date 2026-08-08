from .constants import MAX_OBSTACLE_RADIUS_M, MIN_OBSTACLE_RADIUS_M
from .vector import Vector


class Obstacle:
    """
    A static circular keep-out volume (building, no-fly zone, terrain).

    Deliberately not an :class:`~interception.actor.Actor`: it has no mass,
    no velocity, and never steps, so it stays out of the actor list.

    ``pos`` and ``radius_m`` default to draws from the world RNG; passing
    them explicitly places an obstacle without consuming any RNG state,
    which is what lets tests pin the geometry.
    """

    def __init__(self, state_ref, pos=None, radius_m=None):
        rng = state_ref.rng
        self.radius_m = (
            float(radius_m)
            if radius_m is not None
            else rng.uniform(MIN_OBSTACLE_RADIUS_M, MAX_OBSTACLE_RADIUS_M)
        )
        if pos is not None:
            self.pos = pos.copy()
        else:
            self.pos = Vector(
                rng.uniform(self.radius_m, state_ref.width - self.radius_m),
                rng.uniform(self.radius_m, state_ref.height - self.radius_m),
            )
