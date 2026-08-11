#include "interception/collision.hpp"

namespace interception {

ClosestApproach closestApproach(const Vec2& p1, const Vec2& p2, const Vec2& q1,
                                const Vec2& q2) {
    const Vec2 dp = q1 - p1;                // relative position at step start
    const Vec2 dv = (q2 - q1) - (p2 - p1);  // relative displacement over step

    const double denom = dv.magnitudeSquared();
    if (denom == 0.0) {
        // No relative motion: separation is constant across the step.
        return {dp.magnitude(), 0.0};
    }

    double t = -dp.dot(dv) / denom;
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    return {(dp + dv * t).magnitude(), t};
}

SweptHitResult sweptHit(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2,
                        double combinedRadius) {
    const ClosestApproach approach = closestApproach(p1, p2, q1, q2);
    return {approach.missDistanceM < combinedRadius, approach.missDistanceM};
}

}  // namespace interception
