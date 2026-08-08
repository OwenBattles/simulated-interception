"""
Continuous collision detection between moving spheres.

A per-step position test misses interceptions at realistic closing speeds:
with a 3 m combined capture radius and a 200 m/s closing rate, the pair
advances 3.3 m per 1/60 s tick and can pass straight through each other
between samples. Everything here works on the swept segment instead, which
also yields the miss distance -- the metric the engagement is judged on.
"""


def closest_approach(p1, p2, q1, q2):
    """
    Closest approach between two points moving linearly over one timestep.

    ``p1``/``p2`` are one body's start and end positions; ``q1``/``q2`` the
    other's. Returns ``(miss_distance_m, t_fraction)`` where ``t_fraction``
    is in [0, 1] and locates the closest approach within the step.
    """
    dp = q1 - p1  # relative position at the start of the step
    dv = (q2 - q1) - (p2 - p1)  # relative displacement over the step

    denom = dv.magnitude_squared()
    if denom == 0.0:
        # No relative motion: separation is constant across the step.
        return dp.magnitude(), 0.0

    t = -dp.dot(dv) / denom
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return (dp + dv * t).magnitude(), t


def swept_hit(p1, p2, q1, q2, combined_radius):
    """
    Test whether two swept spheres touch during the step.

    Returns ``(hit, miss_distance_m)``. ``miss_distance_m`` is reported
    whether or not the test passes, so callers can track how near a failed
    intercept came.
    """
    miss, _ = closest_approach(p1, p2, q1, q2)
    return miss < combined_radius, miss
