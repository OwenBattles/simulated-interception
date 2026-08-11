#pragma once

#include "interception/actor.hpp"

namespace interception {

/// Evader. Its behaviour is a blend of two extremes, set by `evasiveness`:
///
/// 0.0  Reynolds wander -- steer toward a point on a circle projected ahead
///      of the nose, where that point drifts around the circle as a random
///      walk. Non-adversarial: it does not react to being chased.
///
/// 1.0  Beam the nearest interceptor -- fly perpendicular to its line of
///      sight. This is the manoeuvre that actually costs a proportional
///      navigation seeker, because it maximises the sight-line rotation the
///      law has to null. Running straight away does not work: a slower
///      target loses that race by definition.
///
/// Between the two, the desired velocities are interpolated, so the knob
/// reads as "how much attention is this thing paying to the interceptor".
///
/// The wander's random walk is advanced on every tick regardless of the
/// blend, so changing evasiveness does not change how much of the RNG
/// stream is consumed -- the same seed keeps the same world layout and only
/// the evader's behaviour changes.
class Target : public Actor {
public:
    Target(const State& world, const VehicleParams& params, double evasiveness,
           Pcg32& rng);

    Vec2 computeSteeringForce(double dt) override;

    /// Advances the wander random walk. Not const: it consumes RNG.
    Vec2 wanderDesiredVelocity(double dt);

    /// Zero when there is no interceptor to react to.
    Vec2 evadeDesiredVelocity() const;

    /// Nearest interceptor, or nullptr if the field is empty.
    const Actor* nearestThreat() const;

    double evasiveness = 0.0;
    double wanderAngle = 0.0;

private:
    /// The world's generator. The wander consumes draws every tick, so the
    /// evader shares the one seeded stream rather than owning its own.
    Pcg32* rng_ = nullptr;
};

}  // namespace interception
