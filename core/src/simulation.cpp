#include "interception/simulation.hpp"

#include <ostream>
#include <random>
#include <stdexcept>
#include <utility>

#include "interception/telemetry.hpp"

namespace interception {

namespace {

/// Draw an arbitrary seed. Deliberately outside the reproducible stream --
/// it only *chooses* which stream to replay, and the choice is recorded.
std::uint64_t drawSeed() {
    std::random_device device;
    return (static_cast<std::uint64_t>(device()) << 32) ^ device();
}

}  // namespace

std::string episodeEndName(EpisodeEnd end) {
    switch (end) {
        case EpisodeEnd::None:
            return "none";
        case EpisodeEnd::Success:
            return "success";
        case EpisodeEnd::Timeout:
            return "timeout";
    }
    throw std::invalid_argument("unhandled episode end");
}

std::ostream& operator<<(std::ostream& os, EpisodeEnd end) {
    return os << episodeEndName(end);
}

Simulation::Simulation(SimulationConfig config)
    : config_(std::move(config)),
      seed_(config_.hasSeed ? config_.seed : drawSeed()),
      state_(std::make_unique<State>(seed_, config_.scenario)) {
    if (config_.recordTelemetry) {
        telemetry_ = std::make_unique<TelemetryRecorder>();
    }
}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::reset() { reset(seed_); }

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    state_->reset(seed_);
    if (telemetry_) {
        telemetry_ = std::make_unique<TelemetryRecorder>();
    }
    steps_ = 0;
    done_ = false;
    endReason_ = EpisodeEnd::None;
}

void Simulation::step() {
    if (done_) {
        return;
    }
    state_->update(config_.dt);
    ++steps_;
    if (telemetry_) {
        telemetry_->capture(*this);
    }
    if (state_->targets().empty()) {
        done_ = true;
        endReason_ = EpisodeEnd::Success;
    } else if (config_.maxSteps > 0 && steps_ >= config_.maxSteps) {
        done_ = true;
        endReason_ = EpisodeEnd::Timeout;
    }
}

Simulation& Simulation::run() {
    while (!done_) {
        step();
    }
    return *this;
}

Observation Simulation::observation() const {
    Observation obs;
    obs.seed = seed_;
    obs.guidance = guidanceLawName(config_.scenario.guidance.law);
    obs.step = steps_;
    obs.elapsedS = elapsedS();
    obs.done = done_;
    obs.endReason = episodeEndName(endReason_);
    obs.intercepts = state_->intercepts();
    obs.hasMinMissDistance = state_->hasMinMissDistance();
    obs.minMissDistanceM = obs.hasMinMissDistance ? state_->minMissDistanceM() : 0.0;
    obs.deltaVMps = state_->totalDeltaVMps();
    obs.numTargets = static_cast<int>(state_->targets().size());
    obs.numAgents = static_cast<int>(state_->agents().size());
    obs.numObstacles = static_cast<int>(state_->obstacles().size());
    return obs;
}

Simulation runHeadless(std::uint64_t seed, int maxSteps, ScenarioParams scenario,
                       bool recordTelemetry) {
    SimulationConfig config;
    config.hasSeed = true;
    config.seed = seed;
    config.maxSteps = maxSteps;
    config.scenario = std::move(scenario);
    config.recordTelemetry = recordTelemetry;

    Simulation sim(std::move(config));
    sim.run();
    return sim;
}

}  // namespace interception
