#include "interception/agent.hpp"

#include <limits>

#include "interception/state.hpp"
#include "interception/target.hpp"

namespace interception {

Agent::Agent(const State& world, const VehicleParams& params,
             const GuidanceParams& guidanceParams, Pcg32& rng)
    : Actor(world, params, rng), guidance_(makeGuidance(guidanceParams)) {}

Vec2 Agent::computeSteeringForce(double dt) {
    const Vec2 avoidance = obstacleAvoidanceForce();
    if (avoidance.magnitudeSquared() > 0.0) {
        return avoidance;
    }

    const Actor* target = currentTarget();
    if (target == nullptr) {
        return {};  // nothing left to chase: coast
    }
    return guidance_->command(*this, *target, dt);
}

const Actor* Agent::currentTarget() const {
    const Actor* nearest = nullptr;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const Target& target : world_->targets()) {
        const double distance = pos.distTo(target.pos);
        if (distance < bestDistance) {
            bestDistance = distance;
            nearest = &target;
        }
    }
    return nearest;
}

GuidanceDiagnostics Agent::diagnostics() const {
    const Actor* target = currentTarget();
    if (target == nullptr) {
        return {};
    }
    return guidance_->diagnostics(*this, *target);
}

}  // namespace interception
