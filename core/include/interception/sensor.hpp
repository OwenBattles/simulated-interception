#pragma once

#include "interception/obstacle.hpp"
#include "interception/vector.hpp"

namespace interception {

/// Forward-looking collision probe: a disc swept ahead of a vehicle along
/// its velocity vector, used to detect obstacles early enough to steer
/// around them.
///
/// A sensor, not a vehicle. No mass, no dynamics, never in the actor list --
/// it is repositioned from its owner's state each tick.
struct Probe {
    double lookaheadS = 0.0;
    double radiusM = 0.0;
    Vec2 pos;

    /// Reposition ahead of `origin`. The standoff is `lookaheadS * speed`,
    /// so a faster vehicle looks further ahead and keeps a constant
    /// reaction *time* rather than a constant reaction distance.
    void update(const Vec2& origin, const Vec2& forward, double speed) {
        pos = origin + forward * (lookaheadS * speed);
    }

    bool intersects(const Obstacle& obstacle) const {
        return pos.distTo(obstacle.pos) < radiusM + obstacle.radiusM;
    }
};

}  // namespace interception
