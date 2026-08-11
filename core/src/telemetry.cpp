#include "interception/telemetry.hpp"

#include "interception/simulation.hpp"

namespace interception {

namespace {

ActorFrame actorFrame(const Actor& actor) {
    ActorFrame frame;
    frame.x = actor.pos.x;
    frame.y = actor.pos.y;
    frame.vx = actor.vel.x;
    frame.vy = actor.vel.y;
    frame.speedMps = actor.vel.magnitude();
    return frame;
}

}  // namespace

void TelemetryRecorder::capture(const Simulation& sim) {
    const State& state = sim.state();

    TelemetryFrame frame;
    frame.t = sim.elapsedS();
    frame.step = sim.steps();

    frame.agents.reserve(state.agents().size());
    for (const Agent& agent : state.agents()) {
        AgentFrame agentFrame;
        static_cast<ActorFrame&>(agentFrame) = actorFrame(agent);
        agentFrame.deltaVMps = agent.deltaVMps;
        agentFrame.accelMps2 = agent.acc.magnitude();
        if (agent.currentTarget() != nullptr) {
            agentFrame.guidance = agent.diagnostics();
            agentFrame.hasGuidance = true;
        }
        frame.agents.push_back(agentFrame);
    }

    frame.targets.reserve(state.targets().size());
    for (const Target& target : state.targets()) {
        frame.targets.push_back(actorFrame(target));
    }

    frames_.push_back(std::move(frame));
}

}  // namespace interception
