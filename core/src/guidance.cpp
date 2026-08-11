#include "interception/guidance.hpp"

#include <cmath>
#include <stdexcept>

#include "interception/actor.hpp"

namespace interception {

EngagementGeometry engagementGeometry(const Actor& agent, const Actor& target) {
    EngagementGeometry geometry;
    geometry.los = target.pos - agent.pos;
    geometry.rangeM = geometry.los.magnitude();
    if (geometry.rangeM == 0.0) {
        return geometry;
    }

    const Vec2 relativeVel = target.vel - agent.vel;
    geometry.closingSpeedMps = -geometry.los.dot(relativeVel) / geometry.rangeM;
    geometry.losRateRadS =
        geometry.los.cross(relativeVel) / (geometry.rangeM * geometry.rangeM);
    return geometry;
}

Vec2 losNormal(const Vec2& los, double rangeM) {
    if (rangeM == 0.0) {
        return {};
    }
    return Vec2{-los.y, los.x} / rangeM;
}

Vec2 allocate(const Actor& agent, const Vec2& lateralForce, double dt) {
    const Vec2 lateral = lateralForce.truncate(agent.maxForce);
    const double spareSquared =
        agent.maxForce * agent.maxForce - lateral.magnitudeSquared();
    const double spare = spareSquared > 0.0 ? std::sqrt(spareSquared) : 0.0;

    const double speedError = agent.maxSpeed - agent.vel.magnitude();
    double axial = agent.mass * speedError / dt;
    axial = axial < -spare ? -spare : (axial > spare ? spare : axial);
    return lateral + agent.forwardVec * axial;
}

GuidanceDiagnostics GuidanceLaw::diagnostics(const Actor& agent,
                                             const Actor& target) const {
    const EngagementGeometry geometry = engagementGeometry(agent, target);
    GuidanceDiagnostics data;
    data.rangeM = geometry.rangeM;
    data.closingSpeedMps = geometry.closingSpeedMps;
    data.losRateRadS = geometry.losRateRadS;
    return data;
}

Vec2 PurePursuit::command(const Actor& agent, const Actor& target, double dt) const {
    const Vec2 desiredVel = (target.pos - agent.pos).setMagnitude(agent.maxSpeed);
    return agent.forceToReach(desiredVel, dt).truncate(agent.maxForce);
}

Vec2 LeadPursuit::aimPoint(const Actor& agent, const Actor& target) const {
    const double timeToGoS = agent.pos.distTo(target.pos) / agent.maxSpeed;
    return target.pos + target.vel * timeToGoS;
}

Vec2 LeadPursuit::command(const Actor& agent, const Actor& target, double dt) const {
    const Vec2 desiredVel =
        (aimPoint(agent, target) - agent.pos).setMagnitude(agent.maxSpeed);
    return agent.forceToReach(desiredVel, dt).truncate(agent.maxForce);
}

Vec2 ProportionalNavigation::lateralAccel(const Actor& agent,
                                          const Actor& target) const {
    const EngagementGeometry geometry = engagementGeometry(agent, target);
    if (geometry.rangeM == 0.0) {
        return {};
    }
    return losNormal(geometry.los, geometry.rangeM) *
           (navConstant_ * geometry.closingSpeedMps * geometry.losRateRadS);
}

Vec2 ProportionalNavigation::command(const Actor& agent, const Actor& target,
                                     double dt) const {
    const EngagementGeometry geometry = engagementGeometry(agent, target);
    if (geometry.rangeM == 0.0) {
        return {};
    }
    if (geometry.closingSpeedMps <= 0.0) {
        // Opening range flips the sign of the PN command and steers the
        // interceptor further away. Fall back to pursuit until closure is
        // re-established -- real seekers gate on closing velocity for the
        // same reason.
        return PurePursuit{}.command(agent, target, dt);
    }
    return allocate(agent, lateralAccel(agent, target) * agent.mass, dt);
}

GuidanceDiagnostics ProportionalNavigation::diagnostics(const Actor& agent,
                                                        const Actor& target) const {
    GuidanceDiagnostics data = GuidanceLaw::diagnostics(agent, target);
    data.lateralAccelMps2 = lateralAccel(agent, target).magnitude();
    data.hasLateralAccel = true;
    return data;
}

Vec2 AugmentedProportionalNavigation::lateralAccel(const Actor& agent,
                                                   const Actor& target) const {
    const Vec2 base = ProportionalNavigation::lateralAccel(agent, target);
    const EngagementGeometry geometry = engagementGeometry(agent, target);
    if (geometry.rangeM == 0.0) {
        return base;
    }
    const Vec2 normal = losNormal(geometry.los, geometry.rangeM);
    // Feed-forward half the target's LOS-normal acceleration. Assumes the
    // target's acceleration is observable, which holds only under this
    // simulator's perfect-information assumption.
    return base + normal * (0.5 * navConstant_ * target.acc.dot(normal));
}

std::unique_ptr<GuidanceLaw> makeGuidance(const GuidanceParams& params) {
    switch (params.law) {
        case GuidanceLawKind::Pursuit:
            return std::make_unique<PurePursuit>();
        case GuidanceLawKind::Lead:
            return std::make_unique<LeadPursuit>();
        case GuidanceLawKind::ProportionalNavigation:
            return std::make_unique<ProportionalNavigation>(params.navConstant);
        case GuidanceLawKind::AugmentedPn:
            return std::make_unique<AugmentedProportionalNavigation>(
                params.navConstant);
    }
    throw std::invalid_argument("unhandled guidance law");
}

}  // namespace interception
