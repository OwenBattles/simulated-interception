#pragma once

#include <memory>
#include <string>

#include "interception/params.hpp"
#include "interception/vector.hpp"

namespace interception {

class Actor;

/// Interceptor guidance laws.
///
/// Four laws share one interface so they can be swapped at runtime and
/// compared on the same seeds:
///
/// pursuit  Pure pursuit. Point at where the target *is*. Always ends in a
///          tail chase and needs increasing lateral acceleration as range
///          closes. The naive baseline.
/// lead     Point at where the target *will be*, from a first-order
///          time-to-go. Exact head-on, degrades in a crossing geometry.
/// pn       Proportional navigation, a = N * Vc * lambda_dot, normal to the
///          line of sight. A collision course is exactly the condition
///          lambda_dot == 0, so PN nulls the rotation of the sight line
///          instead of predicting a position. Near-optimal in control
///          effort, and what real interceptors fly.
/// apn      Augmented PN. Adds N/2 of the target's LOS-normal acceleration
///          as feed-forward, recovering the lag PN shows against a
///          manoeuvring target.
///
/// All of them return a force in newtons that is already within the
/// airframe limit. Actor::integrate clamps again as a backstop, but a law
/// that returns an unrealizable command hides how much authority it wanted
/// and makes laws incomparable -- one that saturates by 40x would look
/// identical to one that just reaches the limit.

struct EngagementGeometry {
    Vec2 los;
    double rangeM = 0.0;
    /// Positive while range is shrinking.
    double closingSpeedMps = 0.0;
    /// Angular rate of the sight line, from the z-component of r x v_rel
    /// over R^2.
    double losRateRadS = 0.0;
};

EngagementGeometry engagementGeometry(const Actor& agent, const Actor& target);

/// Unit vector normal to the sight line (the sight line rotated +90 deg).
Vec2 losNormal(const Vec2& los, double rangeM);

/// Split the force budget between turning and accelerating.
///
/// Guidance only commands a *lateral* acceleration -- it says nothing about
/// throttle. Left alone the interceptor would fly the whole engagement at
/// whatever speed it spawned with, so any budget the turn does not consume
/// is spent closing to top speed. Turning has priority: a slower
/// interceptor on a collision course beats a fast one that cannot correct.
Vec2 allocate(const Actor& agent, const Vec2& lateralForce, double dt);

struct GuidanceDiagnostics {
    double rangeM = 0.0;
    double closingSpeedMps = 0.0;
    double losRateRadS = 0.0;
    double lateralAccelMps2 = 0.0;
    bool hasLateralAccel = false;
};

class GuidanceLaw {
public:
    virtual ~GuidanceLaw() = default;

    virtual GuidanceLawKind kind() const = 0;
    std::string name() const { return toString(kind()); }

    virtual Vec2 command(const Actor& agent, const Actor& target, double dt) const = 0;
    virtual GuidanceDiagnostics diagnostics(const Actor& agent,
                                            const Actor& target) const;
};

class PurePursuit : public GuidanceLaw {
public:
    GuidanceLawKind kind() const override { return GuidanceLawKind::Pursuit; }
    Vec2 command(const Actor& agent, const Actor& target, double dt) const override;
};

class LeadPursuit : public GuidanceLaw {
public:
    GuidanceLawKind kind() const override { return GuidanceLawKind::Lead; }
    Vec2 aimPoint(const Actor& agent, const Actor& target) const;
    Vec2 command(const Actor& agent, const Actor& target, double dt) const override;
};

class ProportionalNavigation : public GuidanceLaw {
public:
    explicit ProportionalNavigation(double navConstant = 4.0)
        : navConstant_(navConstant) {}

    GuidanceLawKind kind() const override {
        return GuidanceLawKind::ProportionalNavigation;
    }

    virtual Vec2 lateralAccel(const Actor& agent, const Actor& target) const;
    Vec2 command(const Actor& agent, const Actor& target, double dt) const override;
    GuidanceDiagnostics diagnostics(const Actor& agent,
                                    const Actor& target) const override;

    double navConstant() const { return navConstant_; }

protected:
    double navConstant_ = 4.0;
};

class AugmentedProportionalNavigation : public ProportionalNavigation {
public:
    explicit AugmentedProportionalNavigation(double navConstant = 4.0)
        : ProportionalNavigation(navConstant) {}

    GuidanceLawKind kind() const override { return GuidanceLawKind::AugmentedPn; }
    Vec2 lateralAccel(const Actor& agent, const Actor& target) const override;
};

std::unique_ptr<GuidanceLaw> makeGuidance(const GuidanceParams& params);

}  // namespace interception
