// Behavioural tests for the evader's adversarial blend.

#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "helpers.hpp"
#include "interception/simulation.hpp"
#include "interception/state.hpp"

using namespace interception;
using namespace interception::test;

namespace {

ScenarioParams scenarioWith(double evasiveness, int agents = 1, int targets = 1) {
    ScenarioParams scenario;
    scenario.numAgents = agents;
    scenario.numTargets = targets;
    scenario.evasiveness = evasiveness;
    return scenario;
}

/// Success rate over a seed sweep -- the metric the knob is meant to move.
double successRate(double evasiveness, int seeds = 120, int maxSteps = 6000) {
    int successes = 0;
    for (int seed = 0; seed < seeds; ++seed) {
        const Simulation sim = runHeadless(seed, maxSteps, scenarioWith(evasiveness));
        if (sim.endReason() == EpisodeEnd::Success) {
            ++successes;
        }
    }
    return static_cast<double>(successes) / seeds;
}

double meanMissDistance(double evasiveness, int seeds = 120, int maxSteps = 6000) {
    double total = 0.0;
    int counted = 0;
    for (int seed = 0; seed < seeds; ++seed) {
        const Simulation sim = runHeadless(seed, maxSteps, scenarioWith(evasiveness));
        if (sim.state().hasMinMissDistance()) {
            total += sim.state().minMissDistanceM();
            ++counted;
        }
    }
    return counted == 0 ? 0.0 : total / counted;
}

}  // namespace

TEST_CASE("evasiveness defaults to the non-adversarial evader") {
    CHECK(ScenarioParams{}.evasiveness == 0.0);
}

TEST_CASE("evasiveness does not change world generation") {
    // The wander's random walk is advanced whatever the blend, so the same
    // seed must lay out the same world -- only behaviour differs. Without
    // that, moving the slider would also reshuffle the obstacles and there
    // would be no way to see what the knob actually did.
    State calm(5, scenarioWith(0.0));
    State evasive(5, scenarioWith(1.0));

    REQUIRE(calm.obstacles().size() == evasive.obstacles().size());
    for (std::size_t i = 0; i < calm.obstacles().size(); ++i) {
        CHECK(calm.obstacles()[i].pos.x == evasive.obstacles()[i].pos.x);
        CHECK(calm.obstacles()[i].pos.y == evasive.obstacles()[i].pos.y);
        CHECK(calm.obstacles()[i].radiusM == evasive.obstacles()[i].radiusM);
    }
    CHECK(calm.targets()[0].pos.x == evasive.targets()[0].pos.x);
    CHECK(calm.agents()[0].pos.x == evasive.agents()[0].pos.x);
}

TEST_CASE("a calm evader ignores the interceptor entirely") {
    // At evasiveness 0 the target's command must not depend on where the
    // interceptor is; that is what "non-adversarial" means.
    State state(5, scenarioWith(0.0));
    Target& target = state.targets()[0];
    const double angleBefore = target.wanderAngle;

    const Vec2 command = target.computeSteeringForce(constants::kSimDt);

    // Same target state, same RNG position, interceptor teleported.
    State moved(5, scenarioWith(0.0));
    moved.agents()[0].pos = Vec2{10.0, 10.0};
    Target& movedTarget = moved.targets()[0];
    REQUIRE(movedTarget.wanderAngle == angleBefore);
    const Vec2 movedCommand = movedTarget.computeSteeringForce(constants::kSimDt);

    CHECK(command.x == doctest::Approx(movedCommand.x));
    CHECK(command.y == doctest::Approx(movedCommand.y));
}

TEST_CASE("a fully evasive target does react to the interceptor") {
    State state(5, scenarioWith(1.0));
    const Vec2 command = state.targets()[0].computeSteeringForce(constants::kSimDt);

    State moved(5, scenarioWith(1.0));
    moved.agents()[0].pos = Vec2{10.0, 10.0};
    const Vec2 movedCommand =
        moved.targets()[0].computeSteeringForce(constants::kSimDt);

    CHECK(command.distTo(movedCommand) > 1.0);
}

TEST_CASE("evade flies a drag: part run, part beam") {
    // 135 degrees off the sight line. Neither pure behaviour is a general
    // evasion -- beaming alone hands a pure pursuer its full closing speed,
    // running alone hands a predictive law a clean collision triangle.
    State state(5, scenarioWith(1.0));
    Target& target = state.targets()[0];
    place(target, {500, 500}, {0, 80});
    place(state.agents()[0], {900, 500}, {-120, 0});

    const Vec2 desired = target.evadeDesiredVelocity();
    REQUIRE(desired.magnitude() == doctest::Approx(target.maxSpeed));

    const Vec2 sightLine = (state.agents()[0].pos - target.pos).normalize();
    const Vec2 normal{-sightLine.y, sightLine.x};

    // Opening range, so a pure pursuer loses closing speed...
    CHECK(desired.dot(sightLine) < 0.0);
    // ...while still crossing the sight line, so a predictive law has LOS
    // rotation to chase.
    CHECK(std::abs(desired.dot(normal)) > 0.0);
    // Equal weights put the two components at the same magnitude.
    CHECK(std::abs(desired.dot(sightLine)) ==
          doctest::Approx(std::abs(desired.dot(normal))));
}

TEST_CASE("evade commits to one side rather than chattering") {
    State state(5, scenarioWith(1.0));
    Target& target = state.targets()[0];
    place(target, {500, 500}, {0, 80});
    place(state.agents()[0], {900, 500}, {-120, 0});

    const Vec2 first = target.evadeDesiredVelocity();
    // The chosen side follows current velocity, so it must be stable while
    // the geometry is.
    CHECK(first.dot(target.vel) >= 0.0);
    const Vec2 again = target.evadeDesiredVelocity();
    CHECK(first.x == doctest::Approx(again.x));
    CHECK(first.y == doctest::Approx(again.y));
}

TEST_CASE("evade is inert when there is nothing to evade") {
    ScenarioParams scenario = scenarioWith(1.0, 0, 1);  // no interceptors
    State state(5, scenario);
    CHECK(state.targets()[0].evadeDesiredVelocity() == Vec2{});
    // And the target still flies rather than stalling.
    state.update(constants::kSimDt);
    CHECK(state.targets()[0].vel.magnitude() > 0.0);
}

TEST_CASE("raising evasiveness costs the interceptor") {
    // The whole point of the knob: probability of kill stops being a
    // constant and becomes a curve.
    const double calmRate = successRate(0.0);
    const double evasiveRate = successRate(1.0);
    CHECK(calmRate == doctest::Approx(1.0));
    CHECK(evasiveRate < calmRate);

    CHECK(meanMissDistance(1.0) > meanMissDistance(0.0));
}
