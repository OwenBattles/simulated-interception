// Behavioural tests for the guidance laws.
//
// These pin the properties that make PN worth having, not just that the
// code runs: LOS rate driven to zero, a straight terminal course, and lower
// control effort than pursuit against the same geometry.

#include <doctest/doctest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "helpers.hpp"
#include "interception/agent.hpp"
#include "interception/guidance.hpp"
#include "interception/state.hpp"

using namespace interception;
using namespace interception::test;

namespace {

struct Flight {
    std::vector<double> losRates;
    std::vector<double> headings;
    double finalRangeM = 0.0;
    double deltaVMps = 0.0;
    bool intercepted = false;
};

/// Fly one interceptor against a coasting target under `law`.
///
/// The target is a bare Actor, which has no steering behaviour and so
/// holds a constant velocity -- exactly the geometry PN assumes, and the
/// one where the textbook ordering should appear.
Flight fly(const GuidanceLaw& law, const Vec2& targetPos, const Vec2& targetVel,
           int maxSteps = 6000, double dt = 1.0 / 60) {
    State state(0, bareScenario(1, 1));
    Agent& agent = state.agents()[0];
    Actor target(state, defaultTarget(), state.rng());

    place(agent, {5000, 5000}, {120, 0});
    place(target, targetPos, targetVel);

    Flight flight;
    for (int i = 0; i < maxSteps; ++i) {
        const EngagementGeometry geometry = engagementGeometry(agent, target);
        flight.losRates.push_back(std::abs(geometry.losRateRadS));
        flight.headings.push_back(agent.forwardVec.angle());
        flight.finalRangeM = geometry.rangeM;

        if (geometry.rangeM < agent.hitRadiusM + target.hitRadiusM) {
            flight.intercepted = true;
            break;
        }

        agent.integrate(law.command(agent, target, dt), dt);
        agent.reorient();
        target.step(dt);  // base Actor coasts
    }
    flight.deltaVMps = agent.deltaVMps;
    return flight;
}

}  // namespace

TEST_CASE("engagement geometry on a head-on closing pair") {
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {100, 0});
    place(target, {1000, 0}, {-50, 0});

    const EngagementGeometry geometry = engagementGeometry(agent, target);
    CHECK(geometry.rangeM == doctest::Approx(1000.0));
    CHECK(geometry.closingSpeedMps == doctest::Approx(150.0));  // 100 + 50
    // Pure head-on: the bearing does not rotate.
    CHECK(geometry.losRateRadS == doctest::Approx(0.0));
}

TEST_CASE("crossing geometry has a nonzero LOS rate") {
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {100, 0});
    place(target, {100, 0}, {0, 50});

    const EngagementGeometry geometry = engagementGeometry(agent, target);
    CHECK(geometry.closingSpeedMps == doctest::Approx(100.0));
    CHECK(geometry.losRateRadS == doctest::Approx(0.5));
}

TEST_CASE("LOS normal is perpendicular and unit length") {
    const Vec2 los{30, 40};
    const Vec2 normal = losNormal(los, los.magnitude());
    CHECK(normal.magnitude() == doctest::Approx(1.0));
    CHECK(normal.dot(los) == doctest::Approx(0.0));
}

TEST_CASE("zero-range geometry is safe") {
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {1, 1}, {0, 0});
    place(target, {1, 1}, {0, 0});

    const EngagementGeometry geometry = engagementGeometry(agent, target);
    CHECK(geometry.rangeM == 0.0);
    CHECK(geometry.closingSpeedMps == 0.0);
    CHECK(geometry.losRateRadS == 0.0);
}

TEST_CASE("PN drives LOS rate toward zero") {
    // The defining property: a collision course is lambda_dot == 0.
    const Flight flight = fly(ProportionalNavigation{4.0}, {6000, 5000}, {0, 60});
    REQUIRE(flight.intercepted);
    CHECK(flight.losRates.front() > 0.01);
    CHECK(flight.losRates.back() < flight.losRates.front() / 10.0);
}

TEST_CASE("pure pursuit lets LOS rate grow") {
    // The contrast case: pursuit converts into a tail chase, so the sight
    // line spins faster as range closes.
    const Flight flight = fly(PurePursuit{}, {6000, 5000}, {0, 60});
    REQUIRE(flight.intercepted);
    CHECK(flight.losRates.back() > flight.losRates.front());
}

TEST_CASE("PN flies a straight terminal course") {
    const Flight pn = fly(ProportionalNavigation{4.0}, {6000, 5000}, {0, 60});
    const Flight pursuit = fly(PurePursuit{}, {6000, 5000}, {0, 60});
    REQUIRE(pn.headings.size() > 31);
    REQUIRE(pursuit.headings.size() > 31);

    const auto drift = [](const std::vector<double>& headings) {
        return std::abs(headings.back() - headings[headings.size() - 31]);
    };
    CHECK(drift(pn.headings) < 0.01);
    CHECK(drift(pn.headings) < drift(pursuit.headings) / 10.0);
}

TEST_CASE("PN spends less control effort than pursuit") {
    const Flight pn = fly(ProportionalNavigation{4.0}, {6000, 5000}, {0, 60});
    const Flight pursuit = fly(PurePursuit{}, {6000, 5000}, {0, 60});
    CHECK(pn.deltaVMps < pursuit.deltaVMps);
}

TEST_CASE("APN reduces to PN against a non-manoeuvring target") {
    // The augmentation term is proportional to target acceleration, so it
    // must vanish exactly when the target is not accelerating.
    const Flight pn = fly(ProportionalNavigation{4.0}, {6000, 5000}, {0, 60});
    const Flight apn =
        fly(AugmentedProportionalNavigation{4.0}, {6000, 5000}, {0, 60});
    CHECK(apn.deltaVMps == doctest::Approx(pn.deltaVMps));
    REQUIRE(apn.losRates.size() == pn.losRates.size());
    for (std::size_t i = 0; i < pn.losRates.size(); ++i) {
        CHECK(apn.losRates[i] == doctest::Approx(pn.losRates[i]));
    }
}

TEST_CASE("APN diverges from PN when the target accelerates") {
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {120, 0});
    place(target, {1000, 200}, {0, 60});
    target.acc = {0, 80};  // pulling away laterally

    const ProportionalNavigation pn{4.0};
    const AugmentedProportionalNavigation apn{4.0};
    CHECK(pn.lateralAccel(agent, target).magnitude() !=
          doctest::Approx(apn.lateralAccel(agent, target).magnitude()));
}

TEST_CASE("PN falls back to pursuit when range is opening") {
    // With negative closing velocity the PN command changes sign and
    // steers away, so the law must gate on closure.
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {-120, 0});  // flying away
    place(target, {1000, 0}, {200, 0});

    CHECK(engagementGeometry(agent, target).closingSpeedMps < 0.0);

    const Vec2 pnForce = ProportionalNavigation{4.0}.command(agent, target, 1.0 / 60);
    const Vec2 pursuitForce = PurePursuit{}.command(agent, target, 1.0 / 60);
    CHECK(pnForce.x == doctest::Approx(pursuitForce.x));
    CHECK(pnForce.y == doctest::Approx(pursuitForce.y));
}

TEST_CASE("commanded force never exceeds the airframe limit") {
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {5, 0});  // far below max speed
    place(target, {50, 40}, {-80, 0});

    for (const std::string& name : {"pursuit", "lead", "pn", "apn"}) {
        GuidanceParams params;
        params.law = guidanceLawFromString(name);
        const auto law = makeGuidance(params);
        const Vec2 force = law->command(agent, target, 1.0 / 60);
        CHECK(force.magnitude() <= agent.maxForce * (1 + 1e-9));
    }
}

TEST_CASE("allocation spends spare budget on closing to top speed") {
    // A head-on geometry needs no turn, so the whole budget goes axial.
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {40, 0});
    place(target, {1000, 0}, {-50, 0});

    const Vec2 force = ProportionalNavigation{4.0}.command(agent, target, 1.0 / 60);
    CHECK(force.dot(agent.forwardVec) > 0.0);
    CHECK(force.magnitude() == doctest::Approx(agent.maxForce));
}

TEST_CASE("makeGuidance builds each law and passes the nav constant") {
    for (const std::string& name : {"pursuit", "lead", "pn", "apn"}) {
        GuidanceParams params;
        params.law = guidanceLawFromString(name);
        params.navConstant = 3.5;
        const auto law = makeGuidance(params);
        CHECK(law->name() == name);
        if (auto* pn = dynamic_cast<ProportionalNavigation*>(law.get())) {
            CHECK(pn->navConstant() == doctest::Approx(3.5));
        }
    }
}

TEST_CASE("unknown guidance law is rejected at parse time") {
    CHECK_THROWS_AS(guidanceLawFromString("telepathy"), std::invalid_argument);
}

TEST_CASE("nav constant scales the command") {
    State state(0, bareScenario(1, 1));
    Actor agent(state, defaultInterceptor(), state.rng());
    Actor target(state, defaultTarget(), state.rng());
    place(agent, {0, 0}, {120, 0});
    place(target, {1000, 100}, {0, 60});

    const double low = ProportionalNavigation{3.0}.lateralAccel(agent, target).magnitude();
    const double high = ProportionalNavigation{5.0}.lateralAccel(agent, target).magnitude();
    CHECK(high > low);
}
