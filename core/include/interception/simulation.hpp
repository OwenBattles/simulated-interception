#pragma once

#include <cstdint>
#include <iosfwd>
#include <memory>
#include <string>

#include "interception/constants.hpp"
#include "interception/params.hpp"
#include "interception/state.hpp"

namespace interception {

class TelemetryRecorder;

enum class EpisodeEnd { None, Success, Timeout };
std::string episodeEndName(EpisodeEnd end);

/// Scoped enums have no default stream insertion, which leaves logs and
/// test failure messages showing an opaque integer.
std::ostream& operator<<(std::ostream& os, EpisodeEnd end);

/// Everything needed to reproduce an episode exactly.
struct SimulationConfig {
    double dt = constants::kSimDt;
    /// 0 = no cap (interactive); > 0 enforces Timeout.
    int maxSteps = 0;
    /// When false, a seed is drawn at construction and recorded, so an
    /// unseeded run is still reportable and replayable after the fact.
    bool hasSeed = false;
    std::uint64_t seed = 0;
    ScenarioParams scenario{};
    bool recordTelemetry = false;
};

/// Flat, serialisable episode summary for logging and Monte Carlo.
struct Observation {
    std::uint64_t seed = 0;
    std::string guidance;
    int step = 0;
    double elapsedS = 0.0;
    bool done = false;
    std::string endReason;
    int intercepts = 0;
    bool hasMinMissDistance = false;
    double minMissDistanceM = 0.0;
    double deltaVMps = 0.0;
    int numTargets = 0;
    int numAgents = 0;
    int numObstacles = 0;
};

/// Owns world state and the episode lifecycle: reset -> step until terminal.
class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    // Movable even though State is not: State lives on the heap, so moving
    // a Simulation moves the pointer and leaves every actor's back-pointer
    // valid. Declared here and defaulted in the .cpp, where the telemetry
    // recorder is a complete type.
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    /// Rebuild the world. Replays the current seed.
    void reset();
    void reset(std::uint64_t seed);

    /// Advance one fixed timestep of `dt` seconds.
    void step();

    /// Step until terminal. Requires maxSteps > 0 to be bounded.
    Simulation& run();

    Observation observation() const;

    State& state() { return *state_; }
    const State& state() const { return *state_; }

    std::uint64_t seed() const { return seed_; }
    int steps() const { return steps_; }
    double dt() const { return config_.dt; }
    double elapsedS() const { return steps_ * config_.dt; }
    bool done() const { return done_; }
    EpisodeEnd endReason() const { return endReason_; }
    const SimulationConfig& config() const { return config_; }

    /// Null unless recordTelemetry was set.
    TelemetryRecorder* telemetry() { return telemetry_.get(); }
    const TelemetryRecorder* telemetry() const { return telemetry_.get(); }

private:
    SimulationConfig config_;
    std::uint64_t seed_ = 0;
    std::unique_ptr<State> state_;
    std::unique_ptr<TelemetryRecorder> telemetry_;
    int steps_ = 0;
    bool done_ = false;
    EpisodeEnd endReason_ = EpisodeEnd::None;
};

/// Run until Success or Timeout; returns the simulation for inspection.
Simulation runHeadless(std::uint64_t seed,
                       int maxSteps = constants::kDefaultHeadlessMaxSteps,
                       ScenarioParams scenario = {}, bool recordTelemetry = false);

}  // namespace interception
