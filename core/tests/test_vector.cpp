#include <doctest/doctest.h>

#include <cmath>

#include "interception/vector.hpp"

using namespace interception;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST_CASE("vector arithmetic") {
    const Vec2 a{3, 4};
    const Vec2 b{1, 2};
    CHECK(a + b == Vec2{4, 6});
    CHECK(a - b == Vec2{2, 2});
    CHECK(a * 2 == Vec2{6, 8});
    CHECK(2 * a == Vec2{6, 8});
    CHECK(a / 2 == Vec2{1.5, 2});
    CHECK(-a == Vec2{-3, -4});
}

TEST_CASE("magnitude and distance") {
    CHECK(Vec2{3, 4}.magnitude() == doctest::Approx(5.0));
    CHECK(Vec2{3, 4}.magnitudeSquared() == doctest::Approx(25.0));
    CHECK(Vec2{0, 0}.distTo({3, 4}) == doctest::Approx(5.0));
}

TEST_CASE("magnitude avoids std::hypot for cross-language parity") {
    // The engine must agree with the Python reference bit for bit, and
    // CPython's math.hypot is not libm's. Both sides compute sqrt(x^2+y^2).
    const Vec2 v{1673.928247981326, -1322.3015756101292};
    CHECK(v.magnitude() == std::sqrt(v.x * v.x + v.y * v.y));
}

TEST_CASE("fromPolar round-trips through angle") {
    const Vec2 v = Vec2::fromPolar(7.0, 0.9);
    CHECK(v.magnitude() == doctest::Approx(7.0));
    CHECK(v.angle() == doctest::Approx(0.9));
}

TEST_CASE("truncate clamps only when over") {
    CHECK(Vec2{3, 4}.truncate(10.0).magnitude() == doctest::Approx(5.0));
    CHECK(Vec2{3, 4}.truncate(2.5).magnitude() == doctest::Approx(2.5));
}

TEST_CASE("zero vector normalisation is safe") {
    // A stationary actor must not produce NaN when reorienting.
    CHECK(Vec2{0, 0}.normalize() == Vec2{0, 0});
    CHECK(Vec2{0, 0}.setMagnitude(5.0) == Vec2{0, 0});
}

TEST_CASE("perpendicular and rotate") {
    const Vec2 v{1, 0};
    CHECK(v.perpendicular() == Vec2{0, 1});
    CHECK(v.perpendicular().dot(v) == doctest::Approx(0.0));
    const Vec2 rotated = v.rotate(kPi / 2);
    CHECK(rotated.x == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(rotated.y == doctest::Approx(1.0));
}

TEST_CASE("cross product sign identifies the side") {
    CHECK(Vec2{1, 0}.cross({0, 1}) == doctest::Approx(1.0));
    CHECK(Vec2{1, 0}.cross({0, -1}) == doctest::Approx(-1.0));
}
