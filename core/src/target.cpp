#include "interception/target.hpp"

#include <cmath>
#include <limits>

#include "interception/agent.hpp"
#include "interception/constants.hpp"
#include "interception/state.hpp"

namespace interception {

namespace {
// Equal weights put the escape heading 135 degrees off the sight line --
// the classic drag: opening range while still crossing the seeker's nose.
constexpr double kFleeWeight = 1.0;
constexpr double kBeamWeight = 1.0;
}  // namespace

Target::Target(const State& world, const VehicleParams& params, double evasivenessIn,
               Pcg32& rng)
    : Actor(world, params, rng), evasiveness(evasivenessIn), rng_(&rng) {}

Vec2 Target::computeSteeringForce(double dt) {
    const Vec2 avoidance = obstacleAvoidanceForce();
    if (avoidance.magnitudeSquared() > 0.0) {
        return avoidance;
    }

    // Always advanced, whatever the blend, so evasiveness does not shift the
    // RNG stream out from under the rest of the world.
    const Vec2 wander = wanderDesiredVelocity(dt);

    // Exact early-out rather than a blend by zero. At evasiveness 0 this
    // must reproduce the original evader bit for bit -- the golden fixtures
    // recorded from the reference engine depend on it.
    if (evasiveness <= 0.0) {
        return forceToReach(wander, dt);
    }

    const Vec2 evade = evadeDesiredVelocity();
    if (evade.magnitudeSquared() == 0.0) {
        return forceToReach(wander, dt);  // nothing to evade
    }

    // Interpolate the desired velocities, then restore full speed: blending
    // two opposed unit-speed vectors would otherwise ask the evader to slow
    // down, which is not what "half evasive" should mean.
    const Vec2 blended = wander + (evade - wander) * evasiveness;
    return forceToReach(blended.setMagnitude(maxSpeed), dt);
}

Vec2 Target::wanderDesiredVelocity(double dt) {
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

    return (wanderPoint - pos).setMagnitude(maxSpeed);
}

const Actor* Target::nearestThreat() const {
    const Actor* nearest = nullptr;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const Agent& agent : world_->agents()) {
        const double distance = pos.distTo(agent.pos);
        if (distance < bestDistance) {
            bestDistance = distance;
            nearest = &agent;
        }
    }
    return nearest;
}

Vec2 Target::evadeDesiredVelocity() const {
    const Actor* threat = nearestThreat();
    if (threat == nullptr) {
        return {};
    }

    const Vec2 los = threat->pos - pos;
    const double range = los.magnitude();
    if (range == 0.0) {
        return {};
    }

    const Vec2 sightLine = los / range;
    const Vec2 normal{-sightLine.y, sightLine.x};

    // Commit to whichever side the evader is already moving toward. Picking
    // the side afresh each tick makes it chatter across the sight line and
    // go nowhere, which is easier to intercept than either choice.
    const double side = normal.dot(vel) >= 0.0 ? 1.0 : -1.0;

    // Drag geometry: part run, part beam.
    //
    // Neither pure behaviour is a general evasion, and measuring both made
    // that obvious. Beaming -- flying square across the sight line --
    // defeats a predictive law like PN, because it manufactures the very
    // LOS rotation PN is trying to null. But it is the *worst* answer to
    // pure pursuit: zero radial velocity means the pursuer closes at its
    // full speed, so pursuit stayed at 100% however hard the target beamed.
    // Running straight away is the mirror image -- it delays a pursuer but
    // hands a predictive law a clean collision triangle.
    //
    // Splitting the difference costs every law something: the flee
    // component denies closing speed, the beam component denies the
    // prediction.
    const Vec2 away = -sightLine;
    const Vec2 beam = normal * side;
    return (away * kFleeWeight + beam * kBeamWeight).setMagnitude(maxSpeed);
}

}  // namespace interception
