#include <doctest/doctest.h>

#include <cmath>

#include "interception/collision.hpp"

using namespace interception;

TEST_CASE("crossing paths meet mid-step") {
    const ClosestApproach result = closestApproach({0, 0}, {10, 0}, {10, 0}, {0, 0});
    // Approx against an exact zero needs an absolute bound, not a relative one.
    CHECK(std::abs(result.missDistanceM) < 1e-9);
    CHECK(result.tFraction == doctest::Approx(0.5));
}

TEST_CASE("parallel motion holds separation") {
    // Equal velocities mean zero relative motion: separation is constant.
    const ClosestApproach result = closestApproach({0, 0}, {10, 0}, {0, 5}, {10, 5});
    CHECK(result.missDistanceM == doctest::Approx(5.0));
    CHECK(result.tFraction == doctest::Approx(0.0));
}

TEST_CASE("closest approach clamps to the step") {
    // Still closing at the end of the step: the answer is the endpoint,
    // not an extrapolated future.
    const ClosestApproach result = closestApproach({0, 0}, {1, 0}, {100, 0}, {99, 0});
    CHECK(result.tFraction == doctest::Approx(1.0));
    CHECK(result.missDistanceM == doctest::Approx(98.0));
}

TEST_CASE("swept test catches a pass-through that endpoints miss") {
    // Regression for tunnelling. A body crossing in front of a static
    // obstacle starts and ends 5.1 m away, so sampling only the endpoints
    // reports no contact -- but it passes within 1 m mid-step, well inside
    // the 2 m capture radius.
    const Vec2 p1{0, 0};
    const Vec2 p2{10, 0};
    const Vec2 q1{5, 1};
    const Vec2 q2{5, 1};

    CHECK(p1.distTo(q1) > 2.0);
    CHECK(p2.distTo(q2) > 2.0);

    const SweptHitResult result = sweptHit(p1, p2, q1, q2, 2.0);
    CHECK(result.hit);
    CHECK(result.missDistanceM == doctest::Approx(1.0));
}

TEST_CASE("swept test reports miss distance on a clean miss") {
    const SweptHitResult result = sweptHit({0, 0}, {10, 0}, {5, 9}, {5, 9}, 2.0);
    CHECK_FALSE(result.hit);
    CHECK(result.missDistanceM == doctest::Approx(9.0));
}
