"""
Interceptor guidance laws.

Four laws share one interface so they can be swapped at runtime and compared
on the same seeds:

``pursuit``
    Pure pursuit. Point at where the target *is*. Always turns in, always
    ends in a tail chase, and needs increasing lateral acceleration as range
    closes. The naive baseline.

``lead``
    Lead pursuit. Point at where the target *will be*, using a first-order
    time-to-go, ``t_go = range / interceptor max speed``. Exact head-on,
    degrades in a crossing geometry because it assumes the interceptor
    closes at its own top speed regardless of aspect.

``pn``
    Proportional navigation, ``a_cmd = N * Vc * lambda_dot``, applied normal
    to the line of sight. The insight is that a collision course is exactly
    the condition ``lambda_dot == 0``: if the bearing to the target is not
    rotating, the two are converging on the same point. So rather than
    chasing a predicted position, PN simply drives the LOS rate to zero,
    which needs no prediction at all and is near-optimal in control effort.
    This is what real interceptors fly.

``apn``
    Augmented PN. Adds ``N/2`` of the target's LOS-normal acceleration as a
    feed-forward term, which recovers the lag PN shows against a manoeuvring
    target.

All of them return a *force* in newtons, not an acceleration, so the caller
can clamp against the airframe limit uniformly.
"""

import math

from .vector import Vector


def engagement_geometry(agent, target):
    """
    Line-of-sight kinematics for one interceptor/target pair.

    Returns ``(los, range_m, closing_speed_mps, los_rate_rad_s)``.

    ``closing_speed`` is positive while range is shrinking. ``los_rate`` is
    the 2D scalar angular rate of the sight line, from the z-component of
    ``r x v_rel`` over ``R^2``.
    """
    los = target.pos - agent.pos
    range_m = los.magnitude()
    if range_m == 0.0:
        return los, 0.0, 0.0, 0.0

    v_rel = target.vel - agent.vel
    closing_speed = -los.dot(v_rel) / range_m
    los_rate = (los.x * v_rel.y - los.y * v_rel.x) / (range_m * range_m)
    return los, range_m, closing_speed, los_rate


def los_normal(los, range_m):
    """Unit vector normal to the sight line (the sight line rotated +90 deg)."""
    if range_m == 0.0:
        return Vector()
    return Vector(-los.y, los.x) / range_m


def allocate(agent, lateral_force, dt):
    """
    Split the force budget between turning and accelerating.

    Guidance only ever commands a *lateral* acceleration -- it says nothing
    about throttle. Left alone the interceptor would fly the whole
    engagement at whatever speed it spawned with, so any budget the turn
    does not consume is spent closing the gap to top speed. Turning has
    priority: a slower interceptor on a collision course beats a fast one
    that cannot correct.
    """
    lateral_force = lateral_force.truncate(agent.max_force)
    spare_squared = agent.max_force**2 - lateral_force.magnitude_squared()
    spare = math.sqrt(spare_squared) if spare_squared > 0.0 else 0.0

    speed_error = agent.max_speed - agent.vel.magnitude()
    axial = agent.mass * speed_error / dt
    axial = max(-spare, min(spare, axial))
    return lateral_force + agent.forward_vec * axial


class GuidanceLaw:
    """
    Base interface.

    ``command`` returns a force in newtons that is already within the
    airframe limit. ``Actor.integrate`` clamps again as a backstop, but a
    law that returns an unrealizable command is hiding how much authority it
    actually wanted -- and makes laws incomparable, since one that saturates
    by 40x looks identical to one that just reaches the limit.
    """

    name = "base"

    def command(self, agent, target, dt):
        raise NotImplementedError

    def diagnostics(self, agent, target):
        """Per-step telemetry for logging and the dashboard."""
        _, range_m, closing, los_rate = engagement_geometry(agent, target)
        return {
            "range_m": range_m,
            "closing_speed_mps": closing,
            "los_rate_rad_s": los_rate,
        }


class PurePursuit(GuidanceLaw):
    name = "pursuit"

    def command(self, agent, target, dt):
        desired_vel = (target.pos - agent.pos).set_magnitude(agent.max_speed)
        return agent.force_to_reach(desired_vel, dt).truncate(agent.max_force)


class LeadPursuit(GuidanceLaw):
    name = "lead"

    def aim_point(self, agent, target):
        time_to_go_s = agent.pos.dist_to(target.pos) / agent.max_speed
        return target.pos + target.vel * time_to_go_s

    def command(self, agent, target, dt):
        desired_vel = (self.aim_point(agent, target) - agent.pos).set_magnitude(
            agent.max_speed
        )
        return agent.force_to_reach(desired_vel, dt).truncate(agent.max_force)


class ProportionalNavigation(GuidanceLaw):
    name = "pn"

    def __init__(self, nav_constant=4.0):
        self.nav_constant = nav_constant

    def lateral_accel(self, agent, target):
        los, range_m, closing, los_rate = engagement_geometry(agent, target)
        if range_m == 0.0:
            return Vector()
        return los_normal(los, range_m) * (self.nav_constant * closing * los_rate)

    def command(self, agent, target, dt):
        _, range_m, closing, _ = engagement_geometry(agent, target)
        if range_m == 0.0:
            return Vector()
        if closing <= 0.0:
            # Opening range flips the sign of the PN command and steers the
            # interceptor further away. Fall back to pursuit until closure
            # is re-established -- real seekers gate on closing velocity for
            # the same reason.
            return PurePursuit().command(agent, target, dt)
        return allocate(agent, self.lateral_accel(agent, target) * agent.mass, dt)

    def diagnostics(self, agent, target):
        data = super().diagnostics(agent, target)
        data["lateral_accel_mps2"] = self.lateral_accel(agent, target).magnitude()
        return data


class AugmentedProportionalNavigation(ProportionalNavigation):
    name = "apn"

    def lateral_accel(self, agent, target):
        base = super().lateral_accel(agent, target)
        los, range_m, _, _ = engagement_geometry(agent, target)
        if range_m == 0.0:
            return base
        normal = los_normal(los, range_m)
        # Feed-forward half the target's LOS-normal acceleration. Assumes
        # the target's acceleration is observable, which holds only under
        # this simulator's perfect-information assumption.
        return base + normal * (0.5 * self.nav_constant * target.acc.dot(normal))


LAWS = {
    PurePursuit.name: PurePursuit,
    LeadPursuit.name: LeadPursuit,
    ProportionalNavigation.name: ProportionalNavigation,
    AugmentedProportionalNavigation.name: AugmentedProportionalNavigation,
}


def make_guidance(params):
    """Build a guidance law from :class:`~interception.params.GuidanceParams`."""
    try:
        law = LAWS[params.law]
    except KeyError:
        raise ValueError(
            f"unknown guidance law {params.law!r}; choose from {sorted(LAWS)}"
        ) from None
    if issubclass(law, ProportionalNavigation):
        return law(nav_constant=params.nav_constant)
    return law()
