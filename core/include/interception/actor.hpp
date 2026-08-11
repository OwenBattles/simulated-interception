#pragma once

#include "interception/params.hpp"
#include "interception/rng.hpp"
#include "interception/sensor.hpp"
#include "interception/vector.hpp"

namespace interception {

class State;

/// Base class for anything with mass that moves under a steering force.
///
/// Subclasses implement `computeSteeringForce`; integration, orientation,
/// bounds, and obstacle avoidance are shared. Nothing here knows about
/// rendering.
class Actor {
public:
    Actor(const State& world, const VehicleParams& params, Pcg32& rng);
    virtual ~Actor() = default;

    /// Advance one tick of length `dt` seconds.
    void step(double dt);

    /// Newtons of commanded steering force. Overridden by subclasses; the
    /// base actor coasts, so it ignores dt.
    virtual Vec2 computeSteeringForce(double /*dt*/) { return {}; }

    /// Semi-implicit Euler in SI units.
    ///
    /// Force is clamped to the airframe limit, converted to acceleration
    /// through mass, and integrated against `dt` -- so trajectories are
    /// identical whether the sim runs at 60 Hz or 240 Hz.
    void integrate(const Vec2& force, double dt);

    /// Align the body frame with the velocity vector (no sideslip).
    void reorient();

    /// Reflect off the engagement-box walls. Returns true on contact.
    bool enforceBounds();

    /// Steer laterally around the nearest obstacle the probe overlaps.
    /// Returns a zero vector when the path ahead is clear, which callers
    /// use to decide whether avoidance overrides their primary behaviour.
    Vec2 obstacleAvoidanceForce();

    /// Force that would bring the vehicle to `desiredVel` in one tick.
    /// Expressed as mass * delta-v / dt so the result is in newtons and
    /// independent of timestep; the caller's clamp to maxForce is what
    /// makes the manoeuvre take several ticks.
    Vec2 forceToReach(const Vec2& desiredVel, double dt) const;

    // Public state, mirroring the Python reference implementation.
    Vec2 pos;
    Vec2 vel;
    Vec2 acc;
    Vec2 steeringForce;
    /// Position at the start of the current step, for swept collision tests.
    Vec2 prevPos;
    Vec2 forwardVec{1.0, 0.0};
    Vec2 sideVec{0.0, 1.0};

    double mass = 0.0;
    double maxSpeed = 0.0;
    double maxForce = 0.0;
    double hitRadiusM = 0.0;
    /// Accumulated control effort, the usual proxy for propellant spent.
    double deltaVMps = 0.0;

    Probe probe;
    VehicleParams params;

protected:
    const State* world_ = nullptr;
};

}  // namespace interception
