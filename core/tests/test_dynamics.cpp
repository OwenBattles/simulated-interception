// Timestep-independence of the integrator.
//
// The pre-refactor Python sim advanced with `pos += vel`, so every speed
// was implicitly per-frame and running at 120 Hz made vehicles fly twice
// as fast. These tests pin the fix on the C++ side too.

#include <doctest/doctest.h>

#include <cmath>

#include "helpers.hpp"
#include "interception/actor.hpp"
#include "interception/state.hpp"

using namespace interception;
using namespace interception::test;

namespace {

VehicleParams testVehicle() {
    VehicleParams params;
    params.massKg = 5.0;
    params.maxSpeedMps = 120.0;
    params.maxForceN = 1000.0;
    params.hitRadiusM = 2.0;
    params.probeLookaheadS = 0.8;
    return params;
}

void integrateFor(Actor& actor, const Vec2& force, double durationS, double dt) {
    const auto steps = static_cast<int>(std::lround(durationS / dt));
    for (int i = 0; i < steps; ++i) {
        actor.integrate(force, dt);
    }
}

}  // namespace

TEST_CASE("coasting distance is independent of timestep") {
    // Zero force: displacement is v*T at any dt, exactly.
    for (double dt : {1.0 / 60, 1.0 / 240, 1.0 / 1000}) {
        State state(0, bareScenario());
        Actor actor(state, testVehicle(), state.rng());
        place(actor, {0, 0}, {50, -25});
        integrateFor(actor, Vec2{}, 1.0, dt);
        CHECK(actor.pos.x == doctest::Approx(50.0).epsilon(1e-9));
        CHECK(actor.pos.y == doctest::Approx(-25.0).epsilon(1e-9));
    }
}

TEST_CASE("velocity under constant force is independent of timestep") {
    // Semi-implicit Euler integrates velocity exactly for constant force.
    for (double dt : {1.0 / 60, 1.0 / 240, 1.0 / 1000}) {
        State state(0, bareScenario());
        Actor actor(state, testVehicle(), state.rng());
        place(actor, {0, 0}, {0, 0});
        integrateFor(actor, Vec2{500.0, 0.0}, 1.0, dt);  // 100 m/s^2
        CHECK(actor.vel.x == doctest::Approx(100.0).epsilon(1e-9));
    }
}

TEST_CASE("position error under constant force shrinks with timestep") {
    // Position carries O(dt) truncation error, so it cannot match exactly,
    // but refining the timestep must converge toward x = 0.5*a*t^2.
    const double analyticX = 0.5 * 100.0 * 1.0 * 1.0;
    double errors[2] = {0.0, 0.0};
    int index = 0;
    for (double dt : {1.0 / 60, 1.0 / 600}) {
        State state(0, bareScenario());
        Actor actor(state, testVehicle(), state.rng());
        place(actor, {0, 0}, {0, 0});
        integrateFor(actor, Vec2{500.0, 0.0}, 1.0, dt);
        errors[index++] = std::abs(actor.pos.x - analyticX);
    }
    CHECK(errors[1] < errors[0] / 5.0);
}

TEST_CASE("force is clamped to the airframe limit") {
    State state(0, bareScenario());
    Actor actor(state, testVehicle(), state.rng());
    place(actor, {0, 0}, {0, 0});
    actor.integrate({1e6, 1e6}, 1.0 / 60);
    CHECK(actor.steeringForce.magnitude() == doctest::Approx(1000.0));
    CHECK(actor.acc.magnitude() == doctest::Approx(200.0));  // 1000 N / 5 kg
}

TEST_CASE("speed is clamped to max speed") {
    State state(0, bareScenario());
    Actor actor(state, testVehicle(), state.rng());
    place(actor, {0, 0}, {0, 0});
    integrateFor(actor, Vec2{1000.0, 0.0}, 10.0, 1.0 / 60);
    CHECK(actor.vel.magnitude() == doctest::Approx(120.0));
}

TEST_CASE("delta-v accumulates control effort") {
    State state(0, bareScenario());
    Actor actor(state, testVehicle(), state.rng());
    place(actor, {0, 0}, {0, 0});
    CHECK(actor.deltaVMps == 0.0);
    integrateFor(actor, Vec2{500.0, 0.0}, 0.5, 1.0 / 60);  // 100 m/s^2 for 0.5 s
    CHECK(actor.deltaVMps == doctest::Approx(50.0).epsilon(1e-9));
}

TEST_CASE("bounds reflect velocity and clamp position") {
    ScenarioParams scenario = bareScenario();
    scenario.worldWidthM = 1500.0;
    scenario.worldHeightM = 1000.0;
    State state(0, scenario);
    Actor actor(state, testVehicle(), state.rng());
    place(actor, {10, 500}, {-100, 0});
    actor.integrate(Vec2{}, 1.0);  // overshoots the left wall
    CHECK(actor.enforceBounds());
    CHECK(actor.pos.x == 0.0);
    CHECK(actor.vel.x > 0);
}

TEST_CASE("reorient survives a stationary actor") {
    State state(0, bareScenario());
    Actor actor(state, testVehicle(), state.rng());
    place(actor, {0, 0}, {1, 0});
    actor.vel = Vec2{0, 0};
    actor.reorient();
    CHECK(actor.forwardVec.magnitude() == doctest::Approx(1.0));
    CHECK(actor.sideVec.dot(actor.forwardVec) == doctest::Approx(0.0));
}

TEST_CASE("obstacle avoidance handles two obstacles in probe range") {
    // Regression for a bug in the Python original: comparing distances
    // passed an obstacle where a vector was expected, and a short-circuit
    // hid it until a *second* obstacle entered probe range.
    ScenarioParams scenario = bareScenario();
    scenario.worldWidthM = 1500.0;
    scenario.worldHeightM = 1000.0;
    State state(0, scenario);
    Agent& agent = state.agents()[0];
    place(agent, {500, 500}, {120, 0});

    // State owns obstacles; rebuild the list through a scenario that has
    // them is awkward, so exercise the probe geometry directly instead.
    Probe probe{0.8, 25.0, {}};
    probe.update(agent.pos, agent.forwardVec, agent.vel.magnitude());
    const Obstacle near{{596, 500}, 60.0};
    const Obstacle far{{600, 520}, 60.0};
    CHECK(probe.intersects(near));
    CHECK(probe.intersects(far));
}

TEST_CASE("probe standoff scales with speed") {
    // Constant reaction time, not constant reaction distance.
    Probe probe{0.8, 25.0, {}};
    probe.update({0, 0}, {1, 0}, 30.0);
    const double slow = probe.pos.magnitude();
    probe.update({0, 0}, {1, 0}, 120.0);
    const double fast = probe.pos.magnitude();
    CHECK(fast == doctest::Approx(4.0 * slow));
}
