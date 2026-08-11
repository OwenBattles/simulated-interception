#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "helpers.hpp"
#include "interception/simulation.hpp"
#include "interception/telemetry.hpp"

using namespace interception;
using namespace interception::test;

namespace {

/// Everything that must be identical between two runs of the same seed.
std::vector<double> snapshot(const State& state) {
    std::vector<double> out;
    for (const Agent& agent : state.agents()) {
        out.insert(out.end(), {agent.pos.x, agent.pos.y, agent.vel.x, agent.vel.y});
    }
    for (const Target& target : state.targets()) {
        out.insert(out.end(), {target.pos.x, target.pos.y, target.vel.x, target.vel.y});
    }
    for (const Obstacle& obstacle : state.obstacles()) {
        out.insert(out.end(), {obstacle.pos.x, obstacle.pos.y, obstacle.radiusM});
    }
    return out;
}

}  // namespace

TEST_CASE("same seed builds the same world") {
    State a(7);
    State b(7);
    CHECK(snapshot(a) == snapshot(b));
}

TEST_CASE("different seeds build different worlds") {
    State a(7);
    State b(8);
    CHECK(snapshot(a) != snapshot(b));
}

TEST_CASE("reset without a seed replays the same world") {
    // Never reseeds from entropy, so the reproducibility guarantee holds.
    State state(11);
    const std::vector<double> original = snapshot(state);
    for (int i = 0; i < 50; ++i) {
        state.update(constants::kSimDt);
    }
    state.reset();
    CHECK(snapshot(state) == original);
}

TEST_CASE("reset with a seed switches worlds and sticks") {
    State state(11);
    state.reset(12);
    State reference(12);
    CHECK(snapshot(state) == snapshot(reference));

    state.update(constants::kSimDt);
    state.reset();  // replays 12, not 11
    CHECK(snapshot(state) == snapshot(reference));
}

TEST_CASE("obstacles are not actors and spawn inside the world") {
    State state(5);
    REQUIRE_FALSE(state.obstacles().empty());
    for (const Obstacle& obstacle : state.obstacles()) {
        CHECK(obstacle.pos.x >= obstacle.radiusM);
        CHECK(obstacle.pos.x <= state.widthM() - obstacle.radiusM);
        CHECK(obstacle.pos.y >= obstacle.radiusM);
        CHECK(obstacle.pos.y <= state.heightM() - obstacle.radiusM);
    }
}

TEST_CASE("actors stay inside the world") {
    State state(5);
    for (int i = 0; i < 600; ++i) {
        state.update(constants::kSimDt);
        for (const Agent& agent : state.agents()) {
            CHECK(agent.pos.x >= 0.0);
            CHECK(agent.pos.x <= state.widthM());
            CHECK(agent.pos.y >= 0.0);
            CHECK(agent.pos.y <= state.heightM());
        }
    }
}

TEST_CASE("simultaneous intercepts all resolve") {
    // Regression from the Python original: kills were applied by removing
    // from the actor list while the step loop iterated it, which skipped
    // entries. Two targets destroyed on the same tick must both go.
    ScenarioParams scenario;
    scenario.numAgents = 1;
    scenario.numTargets = 3;
    State state(3, scenario);

    Agent& agent = state.agents()[0];
    state.targets()[0].pos = agent.pos;
    state.targets()[1].pos = agent.pos;
    const Vec2 survivorPos = state.targets()[2].pos;

    state.update(constants::kSimDt);

    CHECK(state.intercepts() == 2);
    REQUIRE(state.targets().size() == 1);
    CHECK(state.targets()[0].prevPos == survivorPos);
}

TEST_CASE("min miss distance is tracked") {
    State state(3);
    CHECK_FALSE(state.hasMinMissDistance());
    for (int i = 0; i < 200; ++i) {
        state.update(constants::kSimDt);
    }
    CHECK(state.hasMinMissDistance());
}

TEST_CASE("headless episode terminates") {
    const Simulation sim = runHeadless(0, 5000);
    CHECK(sim.done());
    CHECK((sim.endReason() == EpisodeEnd::Success ||
           sim.endReason() == EpisodeEnd::Timeout));
}

TEST_CASE("step cap produces a timeout") {
    const Simulation sim = runHeadless(0, 5);
    CHECK(sim.endReason() == EpisodeEnd::Timeout);
    CHECK(sim.steps() == 5);
    CHECK_FALSE(sim.state().targets().empty());
}

TEST_CASE("success clears the field") {
    const Simulation sim = runHeadless(0, 5000);
    CHECK(sim.endReason() == EpisodeEnd::Success);
    CHECK(sim.state().targets().empty());
    CHECK(sim.state().intercepts() == 1);
}

TEST_CASE("elapsed time tracks steps and dt") {
    const Simulation sim = runHeadless(0, 100);
    CHECK(sim.elapsedS() == doctest::Approx(100 * sim.dt()));
}

TEST_CASE("same seed reproduces the episode") {
    const Observation a = runHeadless(42, 5000).observation();
    const Observation b = runHeadless(42, 5000).observation();
    CHECK(a.step == b.step);
    CHECK(a.intercepts == b.intercepts);
    CHECK(a.minMissDistanceM == doctest::Approx(b.minMissDistanceM));
    CHECK(a.deltaVMps == doctest::Approx(b.deltaVMps));
}

TEST_CASE("unseeded run records a concrete replayable seed") {
    SimulationConfig config;
    config.maxSteps = 2000;  // hasSeed stays false: draw one
    Simulation sim(config);
    sim.run();

    const Simulation replay = runHeadless(sim.seed(), 2000);
    CHECK(replay.observation().step == sim.observation().step);
    CHECK(replay.observation().deltaVMps ==
          doctest::Approx(sim.observation().deltaVMps));
}

TEST_CASE("stepping a finished episode is a no-op") {
    Simulation sim = runHeadless(0, 5);
    const int steps = sim.steps();
    sim.step();
    CHECK(sim.steps() == steps);
}

TEST_CASE("telemetry is opt-in and does not perturb the trajectory") {
    const Observation plain = runHeadless(0, 200).observation();
    Simulation recorded = runHeadless(0, 200, ScenarioParams{}, true);
    REQUIRE(recorded.telemetry() != nullptr);
    CHECK(static_cast<int>(recorded.telemetry()->frames().size()) == recorded.steps());
    CHECK(recorded.observation().step == plain.step);
    CHECK(recorded.observation().deltaVMps == doctest::Approx(plain.deltaVMps));
}

TEST_CASE("telemetry captures guidance diagnostics while a target lives") {
    Simulation sim = runHeadless(0, 300, ScenarioParams{}, true);
    REQUIRE(sim.telemetry() != nullptr);
    const auto& frames = sim.telemetry()->frames();
    REQUIRE_FALSE(frames.empty());
    CHECK(frames.front().agents.front().hasGuidance);
    CHECK(frames.front().agents.front().guidance.rangeM > 0.0);
}

TEST_CASE("multi-agent multi-target engagement runs") {
    ScenarioParams scenario;
    scenario.numAgents = 3;
    scenario.numTargets = 2;
    const Simulation sim = runHeadless(4, 5000, scenario);
    CHECK(sim.state().agents().size() == 3);
    CHECK(sim.done());
}
