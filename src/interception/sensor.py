from dataclasses import dataclass, field

from .vector import Vector


@dataclass
class Probe:
    """
    Forward-looking collision probe: a disc swept ahead of a vehicle along
    its velocity vector, used to detect obstacles early enough to steer
    around them.

    This is a sensor, not a vehicle. It has no mass, no dynamics, and is
    never added to the actor list -- it is repositioned from its owner's
    state each tick.

    The standoff distance is ``lookahead_s * speed``, so a vehicle flying
    faster automatically looks further ahead and keeps a constant reaction
    time rather than a constant reaction distance.
    """

    lookahead_s: float
    radius_m: float
    pos: Vector = field(default_factory=Vector)

    def update(self, origin, forward_vec, speed):
        """Reposition the probe ahead of ``origin`` along ``forward_vec``."""
        self.pos = origin + forward_vec * (self.lookahead_s * speed)

    def intersects(self, obstacle):
        return self.pos.dist_to(obstacle.pos) < self.radius_m + obstacle.radius_m
