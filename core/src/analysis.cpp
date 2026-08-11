#include "interception/analysis.hpp"

#include <cmath>

#include "interception/actor.hpp"
#include "interception/agent.hpp"
#include "interception/guidance.hpp"
#include "interception/state.hpp"

namespace interception {

namespace {

/// A world with no obstacles and effectively no walls, so the trace
/// measures guidance and nothing else.
ScenarioParams bareScenario(const VehicleParams& interceptor,
                            const VehicleParams& target) {
    ScenarioParams scenario;
    scenario.minObstacles = 0;
    scenario.maxObstacles = 0;
    scenario.numAgents = 1;
    scenario.numTargets = 1;
    scenario.worldWidthM = 1e7;
    scenario.worldHeightM = 1e7;
    scenario.interceptor = interceptor;
    scenario.target = target;
    return scenario;
}

void place(Actor& actor, const Vec2& pos, const Vec2& vel) {
    actor.pos = pos;
    actor.vel = vel;
    actor.prevPos = pos;
    actor.reorient();
}

}  // namespace

GuidanceTrace traceGuidance(const GuidanceTraceRequest& request) {
    State state(0, bareScenario(request.interceptor, request.target));
    Agent& agent = state.agents()[0];

    // A base Actor rather than a Target: Target wanders, and the point of
    // this harness is a coherent, repeatable manoeuvre.
    Actor target(state, request.target, state.rng());

    place(agent, request.agentPos, request.agentVel);
    place(target, request.targetPos, request.targetVel);

    const auto law = makeGuidance(request.guidance);
    const double captureRadius = agent.hitRadiusM + target.hitRadiusM;

    GuidanceTrace trace;
    for (int step = 0; step < request.maxSteps; ++step) {
        const EngagementGeometry geometry = engagementGeometry(agent, target);
        trace.elapsedS = step * request.dt;

        if (geometry.rangeM >= request.plotGateM) {
            trace.points.push_back(
                {trace.elapsedS, std::abs(geometry.losRateRadS), geometry.rangeM});
        }
        if (geometry.rangeM < captureRadius) {
            trace.intercepted = true;
            break;
        }

        agent.integrate(law->command(agent, target, request.dt), request.dt);
        agent.reorient();

        // Steady turn: lateral acceleration applied directly rather than
        // through the force clamp, so the manoeuvre is exactly as specified.
        const Vec2 normal = target.vel.normalize().perpendicular();
        target.acc = normal * request.targetLateralAccelMps2;
        target.vel = target.vel + target.acc * request.dt;
        target.pos = target.pos + target.vel * request.dt;
        target.reorient();
    }

    trace.deltaVMps = agent.deltaVMps;
    return trace;
}

}  // namespace interception
