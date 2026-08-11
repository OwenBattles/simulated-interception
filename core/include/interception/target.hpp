#pragma once

#include "interception/actor.hpp"

namespace interception {

/// Evader flying a Reynolds wander: steer toward a point on a circle
/// projected ahead of the nose, where that point drifts around the circle
/// as a random walk.
///
/// The walk increment scales with sqrt(dt) rather than dt so the path
/// statistics are the same at any timestep -- a plain `* dt` would make the
/// evader smoother simply by running the sim faster.
///
/// This target is non-adversarial: it does not react to being chased.
class Target : public Actor {
public:
    Target(const State& world, const VehicleParams& params, Pcg32& rng);

    Vec2 computeSteeringForce(double dt) override;
    Vec2 wanderForce(double dt);

    double wanderAngle = 0.0;

private:
    /// The world's generator. The wander consumes draws every tick, so the
    /// evader has to share the one seeded stream rather than owning its own.
    Pcg32* rng_ = nullptr;
};

}  // namespace interception
