#pragma once

#include <vector>

#include "interception/guidance.hpp"

namespace interception {

class Simulation;

/// Per-step telemetry capture.
///
/// Simulation::observation() summarises an episode after it ends, which is
/// all a Monte Carlo sweep needs. Anything that plots an engagement -- a
/// guidance comparison, or the dashboard -- needs the time series instead.
///
/// Recording is opt-in: it allocates per entity per step, which measurably
/// slows a large sweep.

struct ActorFrame {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double speedMps = 0.0;
};

struct AgentFrame : ActorFrame {
    double deltaVMps = 0.0;
    double accelMps2 = 0.0;
    GuidanceDiagnostics guidance;
    /// False once the field is clear and there is nothing to engage.
    bool hasGuidance = false;
};

struct TelemetryFrame {
    double t = 0.0;
    int step = 0;
    std::vector<AgentFrame> agents;
    std::vector<ActorFrame> targets;
};

class TelemetryRecorder {
public:
    void capture(const Simulation& sim);

    const std::vector<TelemetryFrame>& frames() const { return frames_; }
    void clear() { frames_.clear(); }

private:
    std::vector<TelemetryFrame> frames_;
};

}  // namespace interception
