import math

from .constants import AVOIDANCE_BRAKING_WEIGHT
from .sensor import Probe
from .vector import Vector


class Actor:
    """
    Base class for anything with mass that moves under a steering force.

    Subclasses implement :meth:`steering_force`; integration, orientation,
    bounds, and obstacle avoidance are shared. This class is pygame-free --
    rendering lives entirely in :mod:`interception.render`.
    """

    def __init__(self, state_ref, params, use_probe=True):
        self.state_ref = state_ref
        self.params = params
        rng = state_ref.rng

        self.pos = Vector(
            rng.uniform(0.0, state_ref.width),
            rng.uniform(0.0, state_ref.height),
        )
        heading = rng.uniform(-math.pi, math.pi)
        self.vel = Vector.from_polar(
            rng.uniform(0.25, 1.0) * params.max_speed_mps, heading
        )
        self.acc = Vector()
        self.steering_force = Vector()

        # Mirrored onto the instance because guidance laws and the renderer
        # read them every tick.
        self.mass = params.mass_kg
        self.max_speed = params.max_speed_mps
        self.max_force = params.max_force_n
        self.hit_radius_m = params.hit_radius_m
        self.probe = (
            Probe(params.probe_lookahead_s, params.probe_radius_m)
            if use_probe
            else None
        )

        self.forward_vec = Vector.from_polar(1.0, heading)
        self.side_vec = self.forward_vec.perpendicular()

        # Position at the start of the current step, for swept collision tests.
        self.prev_pos = self.pos.copy()
        self.delta_v_mps = 0.0

    # --- lifecycle ------------------------------------------------------
    def step(self, dt):
        """Advance one tick of length ``dt`` seconds."""
        self.prev_pos = self.pos.copy()
        self.integrate(self.steering_force_at(dt), dt)
        # Bounds before reorient: a wall bounce reverses velocity, and the
        # body frame must reflect the post-bounce heading, not the one that
        # flew into the wall.
        self.enforce_bounds()
        self.reorient()

    def steering_force_at(self, dt):
        """Newtons of commanded steering force. Overridden by subclasses."""
        return Vector()

    def integrate(self, force, dt):
        """
        Semi-implicit Euler in SI units.

        Force is clamped to the airframe limit, converted to acceleration
        through mass, and integrated against ``dt`` -- so trajectories are
        identical whether the sim runs at 60 Hz or 240 Hz.
        """
        self.steering_force = force.truncate(self.max_force)
        self.acc = self.steering_force / self.mass
        delta_v = self.acc * dt
        self.vel = (self.vel + delta_v).truncate(self.max_speed)
        self.pos = self.pos + self.vel * dt
        # Accumulated control effort, the usual proxy for propellant spent.
        self.delta_v_mps += delta_v.magnitude()

    def reorient(self):
        """Align the body frame with the velocity vector (no sideslip)."""
        if self.vel.magnitude_squared() > 0.0:
            self.forward_vec = self.vel.normalize()
            self.side_vec = self.forward_vec.perpendicular()

    def enforce_bounds(self):
        """Reflect off the engagement-box walls. Returns True on contact."""
        w, h = self.state_ref.width, self.state_ref.height
        hit = False
        if self.pos.x < 0.0:
            self.pos.update(x=0.0)
            self.vel.update(x=-self.vel.x)
            hit = True
        elif self.pos.x > w:
            self.pos.update(x=w)
            self.vel.update(x=-self.vel.x)
            hit = True
        if self.pos.y < 0.0:
            self.pos.update(y=0.0)
            self.vel.update(y=-self.vel.y)
            hit = True
        elif self.pos.y > h:
            self.pos.update(y=h)
            self.vel.update(y=-self.vel.y)
            hit = True
        return hit

    # --- steering behaviours --------------------------------------------
    def obstacle_avoidance_force(self):
        """
        Steer laterally around the nearest obstacle the probe overlaps.

        Returns a zero vector when the path ahead is clear, which callers
        use to decide whether avoidance overrides their primary behaviour.
        """
        if self.probe is None:
            return Vector()

        self.probe.update(self.pos, self.forward_vec, self.vel.magnitude())

        most_threatening = None
        nearest = math.inf
        for obstacle in self.state_ref.obstacles:
            if not self.probe.intersects(obstacle):
                continue
            distance = self.pos.dist_to(obstacle.pos)
            if distance < nearest:
                nearest = distance
                most_threatening = obstacle

        if most_threatening is None:
            return Vector()

        # Turn away from whichever side the obstacle sits on, and bleed a
        # little speed so the turn radius tightens.
        to_obstacle = most_threatening.pos - self.pos
        side_steer = -1.0 if self.side_vec.dot(to_obstacle) > 0.0 else 1.0
        lateral = self.side_vec * (side_steer * self.max_force)
        braking = self.forward_vec * (AVOIDANCE_BRAKING_WEIGHT * self.max_force)
        return lateral - braking

    def force_to_reach(self, desired_vel, dt):
        """
        Force that would bring the vehicle to ``desired_vel`` in one tick.

        Expressed as mass * delta-v / dt so the result is in newtons and
        independent of timestep; the caller's clamp to ``max_force`` is what
        makes the manoeuvre take multiple ticks.
        """
        return (desired_vel - self.vel) * (self.mass / dt)
