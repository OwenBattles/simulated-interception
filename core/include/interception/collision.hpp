#pragma once

#include "interception/vector.hpp"

namespace interception {

/// Continuous collision detection between moving spheres.
///
/// A per-step position test misses interceptions at realistic closing
/// speeds: with a 3 m combined capture radius and a 200 m/s closing rate,
/// the pair advances 3.3 m per 1/60 s tick and can pass straight through
/// each other between samples. Everything here works on the swept segment
/// instead, which also yields the miss distance -- the metric the
/// engagement is judged on.

struct ClosestApproach {
    double missDistanceM = 0.0;
    /// Where in the step the closest approach happened, in [0, 1].
    double tFraction = 0.0;
};

/// Closest approach between two points moving linearly over one timestep.
/// `p1`/`p2` are one body's start and end positions; `q1`/`q2` the other's.
ClosestApproach closestApproach(const Vec2& p1, const Vec2& p2, const Vec2& q1,
                                const Vec2& q2);

struct SweptHitResult {
    bool hit = false;
    /// Reported whether or not the test passes, so callers can track how
    /// near a failed intercept came.
    double missDistanceM = 0.0;
};

SweptHitResult sweptHit(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2,
                        double combinedRadius);

}  // namespace interception
