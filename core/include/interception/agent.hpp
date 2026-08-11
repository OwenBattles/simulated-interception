#pragma once

#include <memory>

#include "interception/actor.hpp"
#include "interception/guidance.hpp"

namespace interception {

/// Interceptor. Steering is delegated to a swappable guidance law;
/// obstacle avoidance pre-empts it whenever the probe is blocked.
class Agent : public Actor {
public:
    Agent(const State& world, const VehicleParams& params,
          const GuidanceParams& guidanceParams, Pcg32& rng);

    Vec2 computeSteeringForce(double dt) override;

    /// Nearest surviving target, or nullptr once the field is clear.
    ///
    /// Each interceptor chooses independently, so several can converge on
    /// the same target. Fleet-level assignment is a separate concern.
    const Actor* currentTarget() const;

    /// Per-step guidance telemetry. `hasLateralAccel` is false and the
    /// values are zero when there is no target left to engage.
    GuidanceDiagnostics diagnostics() const;

    const GuidanceLaw& guidance() const { return *guidance_; }

private:
    std::unique_ptr<GuidanceLaw> guidance_;
};

}  // namespace interception
