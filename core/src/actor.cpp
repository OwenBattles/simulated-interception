#include "interception/actor.hpp"

#include <limits>

#include "interception/state.hpp"

namespace interception {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Actor::Actor(const State& world, const VehicleParams& vehicleParams, Pcg32& rng)
    : params(vehicleParams), world_(&world) {
    mass = params.massKg;
    maxSpeed = params.maxSpeedMps;
    maxForce = params.maxForceN;
    hitRadiusM = params.hitRadiusM;
    probe.lookaheadS = params.probeLookaheadS;
    probe.radiusM = params.probeRadiusM;

    // Each draw is sequenced into its own named local on purpose. C++ does
    // not specify the evaluation order of function arguments, so writing
    // `Vec2{rng.uniform(...), rng.uniform(...)}` would let the compiler
    // consume the stream in either order -- and the world generated from a
    // given seed would stop matching the Python reference, silently and
    // only on some toolchains.
    const double spawnX = rng.uniform(0.0, world.widthM());
    const double spawnY = rng.uniform(0.0, world.heightM());
    pos = {spawnX, spawnY};

    const double heading = rng.uniform(-kPi, kPi);
    const double speedFraction = rng.uniform(0.25, 1.0);
    vel = Vec2::fromPolar(speedFraction * maxSpeed, heading);

    forwardVec = Vec2::fromPolar(1.0, heading);
    sideVec = forwardVec.perpendicular();
    prevPos = pos;
}

void Actor::step(double dt) {
    prevPos = pos;
    integrate(computeSteeringForce(dt), dt);
    // Bounds before reorient: a wall bounce reverses velocity, and the body
    // frame must reflect the post-bounce heading, not the one that flew
    // into the wall.
    enforceBounds();
    reorient();
}

void Actor::integrate(const Vec2& force, double dt) {
    steeringForce = force.truncate(maxForce);
    acc = steeringForce / mass;
    const Vec2 deltaV = acc * dt;
    vel = (vel + deltaV).truncate(maxSpeed);
    pos = pos + vel * dt;
    deltaVMps += deltaV.magnitude();
}

void Actor::reorient() {
    if (vel.magnitudeSquared() > 0.0) {
        forwardVec = vel.normalize();
        sideVec = forwardVec.perpendicular();
    }
}

bool Actor::enforceBounds() {
    const double w = world_->widthM();
    const double h = world_->heightM();
    bool touched = false;

    if (pos.x < 0.0) {
        pos.x = 0.0;
        vel.x = -vel.x;
        touched = true;
    } else if (pos.x > w) {
        pos.x = w;
        vel.x = -vel.x;
        touched = true;
    }

    if (pos.y < 0.0) {
        pos.y = 0.0;
        vel.y = -vel.y;
        touched = true;
    } else if (pos.y > h) {
        pos.y = h;
        vel.y = -vel.y;
        touched = true;
    }

    return touched;
}

Vec2 Actor::obstacleAvoidanceForce() {
    probe.update(pos, forwardVec, vel.magnitude());

    const Obstacle* mostThreatening = nullptr;
    double nearest = std::numeric_limits<double>::infinity();
    for (const Obstacle& obstacle : world_->obstacles()) {
        if (!probe.intersects(obstacle)) {
            continue;
        }
        const double distance = pos.distTo(obstacle.pos);
        if (distance < nearest) {
            nearest = distance;
            mostThreatening = &obstacle;
        }
    }

    if (mostThreatening == nullptr) {
        return {};
    }

    // Turn away from whichever side the obstacle sits on, and bleed a
    // little speed so the turn radius tightens.
    const Vec2 toObstacle = mostThreatening->pos - pos;
    const double sideSteer = sideVec.dot(toObstacle) > 0.0 ? -1.0 : 1.0;
    const Vec2 lateral = sideVec * (sideSteer * maxForce);
    const Vec2 braking = forwardVec * (constants::kAvoidanceBrakingWeight * maxForce);
    return lateral - braking;
}

Vec2 Actor::forceToReach(const Vec2& desiredVel, double dt) const {
    return (desiredVel - vel) * (mass / dt);
}

}  // namespace interception
