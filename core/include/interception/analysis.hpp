#pragma once

#include <vector>

#include "interception/params.hpp"
#include "interception/vector.hpp"

namespace interception {

/// Canonical single-engagement harness used to compare guidance laws.
///
/// The shipped scenario's evader flies a random walk, which cannot
/// distinguish the laws -- nulling the LOS rate only means "collision
/// course" if the target holds a coherent course. This runs the textbook
/// geometry instead: one interceptor against a target in a steady turn, so
/// the laws separate the way the literature says they should.
///
/// It lives in the engine rather than in the plotting script because it is
/// physics, and because a figure whose numbers come from a second
/// implementation is worth nothing.

struct GuidanceTracePoint {
    double t = 0.0;
    /// |lambda_dot|, the quantity PN exists to drive to zero.
    double losRateRadS = 0.0;
    double rangeM = 0.0;
};

struct GuidanceTrace {
    std::vector<GuidanceTracePoint> points;
    bool intercepted = false;
    /// End-to-end flight time, including the gated-out terminal phase.
    double elapsedS = 0.0;
    double deltaVMps = 0.0;
};

struct GuidanceTraceRequest {
    GuidanceParams guidance{};
    double dt = 1.0 / 60.0;
    int maxSteps = 4000;
    /// Stop recording inside this range. lambda_dot carries 1/R^2 and so
    /// diverges in the last fraction of a second no matter how well the law
    /// is doing; plotting that spike compresses every meaningful curve into
    /// the baseline. The flight itself continues to intercept, so the
    /// reported time and delta-v stay end-to-end.
    double plotGateM = 50.0;
    Vec2 agentPos{5000.0, 5000.0};
    Vec2 agentVel{120.0, 0.0};
    Vec2 targetPos{6200.0, 5000.0};
    Vec2 targetVel{0.0, 80.0};
    /// Constant lateral acceleration, i.e. a steady turn.
    double targetLateralAccelMps2 = 30.0;
    VehicleParams interceptor = defaultInterceptor();
    VehicleParams target = defaultTarget();
};

GuidanceTrace traceGuidance(const GuidanceTraceRequest& request);

}  // namespace interception
