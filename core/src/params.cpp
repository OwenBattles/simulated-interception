#include "interception/params.hpp"

#include <stdexcept>

namespace interception {

std::string toString(GuidanceLawKind kind) {
    switch (kind) {
        case GuidanceLawKind::Pursuit:
            return "pursuit";
        case GuidanceLawKind::Lead:
            return "lead";
        case GuidanceLawKind::ProportionalNavigation:
            return "pn";
        case GuidanceLawKind::AugmentedPn:
            return "apn";
    }
    throw std::invalid_argument("unhandled guidance law");
}

GuidanceLawKind guidanceLawFromString(const std::string& name) {
    if (name == "pursuit") return GuidanceLawKind::Pursuit;
    if (name == "lead") return GuidanceLawKind::Lead;
    if (name == "pn") return GuidanceLawKind::ProportionalNavigation;
    if (name == "apn") return GuidanceLawKind::AugmentedPn;
    throw std::invalid_argument(
        "unknown guidance law '" + name + "'; choose from apn, lead, pn, pursuit");
}

VehicleParams defaultInterceptor() {
    VehicleParams p;
    p.massKg = constants::kAgentMassKg;
    p.maxSpeedMps = constants::kAgentMaxSpeedMps;
    p.maxForceN = constants::kAgentMaxForceN;
    p.hitRadiusM = constants::kAgentHitRadiusM;
    p.probeLookaheadS = constants::kAgentProbeLookaheadS;
    p.probeRadiusM = constants::kProbeRadiusM;
    return p;
}

VehicleParams defaultTarget() {
    VehicleParams p;
    p.massKg = constants::kTargetMassKg;
    p.maxSpeedMps = constants::kTargetMaxSpeedMps;
    p.maxForceN = constants::kTargetMaxForceN;
    p.hitRadiusM = constants::kTargetHitRadiusM;
    p.probeLookaheadS = constants::kTargetProbeLookaheadS;
    p.probeRadiusM = constants::kProbeRadiusM;
    return p;
}

}  // namespace interception
