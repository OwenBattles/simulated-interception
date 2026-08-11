#include "interception/target.hpp"

#include <cmath>

#include "interception/constants.hpp"
#include "interception/state.hpp"

namespace interception {

Target::Target(const State& world, const VehicleParams& params, Pcg32& rng)
    : Actor(world, params, rng), rng_(&rng) {}

Vec2 Target::computeSteeringForce(double dt) {
    const Vec2 avoidance = obstacleAvoidanceForce();
    if (avoidance.magnitudeSquared() > 0.0) {
        return avoidance;
    }
    return wanderForce(dt);
}

Vec2 Target::wanderForce(double dt) {
    wanderAngle += rng_->gauss(0.0, 1.0) *
                   (constants::kTargetWanderSigmaRadPerSqrtS * std::sqrt(dt));
    if (wanderAngle < -constants::kTargetWanderMaxRad) {
        wanderAngle = -constants::kTargetWanderMaxRad;
    } else if (wanderAngle > constants::kTargetWanderMaxRad) {
        wanderAngle = constants::kTargetWanderMaxRad;
    }

    const Vec2 circleCentre = pos + forwardVec * constants::kTargetWanderCircleDistM;
    const Vec2 offset = Vec2::fromPolar(constants::kTargetWanderCircleRadiusM,
                                        forwardVec.angle() + wanderAngle);
    const Vec2 wanderPoint = circleCentre + offset;

    const Vec2 desiredVel = (wanderPoint - pos).setMagnitude(maxSpeed);
    return forceToReach(desiredVel, dt);
}

}  // namespace interception
