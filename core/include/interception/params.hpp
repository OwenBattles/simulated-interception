#pragma once

#include <string>

#include "interception/constants.hpp"

namespace interception {

/// Which guidance law an interceptor flies.
///
/// An enum rather than a string so an unknown law is a compile-time or
/// parse-time error rather than a silent fallback at the first tick. The
/// string forms exist only at the boundaries -- CLI, bindings, web API.
enum class GuidanceLawKind { Pursuit, Lead, ProportionalNavigation, AugmentedPn };

std::string guidanceLawName(GuidanceLawKind kind);

/// Throws std::invalid_argument on an unrecognised name.
GuidanceLawKind guidanceLawFromString(const std::string& name);

/// Airframe limits for one vehicle class.
struct VehicleParams {
    double massKg = constants::kAgentMassKg;
    double maxSpeedMps = constants::kAgentMaxSpeedMps;
    double maxForceN = constants::kAgentMaxForceN;
    double hitRadiusM = constants::kAgentHitRadiusM;
    double probeLookaheadS = constants::kAgentProbeLookaheadS;
    double probeRadiusM = constants::kProbeRadiusM;

    double maxAccelMps2() const { return maxForceN / massKg; }
    double maxAccelG() const { return maxAccelMps2() / 9.80665; }

    /// Minimum turn radius at top speed, v^2 / a.
    double turnRadiusM() const {
        return maxSpeedMps * maxSpeedMps / maxAccelMps2();
    }
};

VehicleParams defaultInterceptor();
VehicleParams defaultTarget();

struct GuidanceParams {
    GuidanceLawKind law = GuidanceLawKind::Lead;
    /// N in a = N * Vc * lambda-dot; 3-5 is typical.
    double navConstant = 4.0;
};

/// Everything about the engagement except the RNG seed.
struct ScenarioParams {
    double worldWidthM = constants::kWorldWidthM;
    double worldHeightM = constants::kWorldHeightM;
    int numAgents = 1;
    int numTargets = 1;
    int minObstacles = constants::kMinObstacleCount;
    int maxObstacles = constants::kMaxObstacleCount;
    VehicleParams interceptor = defaultInterceptor();
    VehicleParams target = defaultTarget();
    GuidanceParams guidance{};

    /// How much the evader reacts to being chased, in [0, 1].
    /// 0 = pure random wander (the original, non-adversarial behaviour);
    /// 1 = beam the nearest interceptor. See target.hpp.
    double evasiveness = 0.0;
};

}  // namespace interception
