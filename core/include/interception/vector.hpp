#pragma once

#include <cmath>

namespace interception {

/// 2D vector in world units (metres, or metres/second for velocities).
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    static Vec2 fromPolar(double r, double theta) {
        return {r * std::cos(theta), r * std::sin(theta)};
    }

    // --- arithmetic ---------------------------------------------------
    constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }

    Vec2& operator+=(const Vec2& o) {
        x += o.x;
        y += o.y;
        return *this;
    }

    constexpr bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
    constexpr bool operator!=(const Vec2& o) const { return !(*this == o); }

    // --- queries --------------------------------------------------------
    /// Deliberately not std::hypot. CPython implements math.hypot itself
    /// with Neumaier summation rather than calling libm, so the two
    /// disagree by up to ~5e-13 -- enough, after a few thousand steps, to
    /// flip a discrete decision and diverge from the Python reference.
    /// sqrt is IEEE-754 correctly rounded, so both languages agree exactly.
    double magnitude() const { return std::sqrt(x * x + y * y); }
    constexpr double magnitudeSquared() const { return x * x + y * y; }
    double angle() const { return std::atan2(y, x); }
    constexpr double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double distTo(const Vec2& o) const { return (*this - o).magnitude(); }

    /// z-component of the 3D cross product; the sign tells you which side.
    constexpr double cross(const Vec2& o) const { return x * o.y - y * o.x; }

    // --- transforms (all return new vectors) ----------------------------

    /// Clamp magnitude to `maxValue`, preserving direction.
    Vec2 truncate(double maxValue) const {
        const double mag = magnitude();
        return mag > maxValue ? *this * (maxValue / mag) : *this;
    }

    /// Rescale to `target` length. A zero vector has no direction to
    /// preserve, so it stays zero rather than producing NaN.
    Vec2 setMagnitude(double target) const {
        const double mag = magnitude();
        return mag == 0.0 ? Vec2{} : *this * (target / mag);
    }

    Vec2 normalize() const { return setMagnitude(1.0); }

    constexpr Vec2 perpendicular() const { return {-y, x}; }

    Vec2 rotate(double angleRad) const {
        const double c = std::cos(angleRad);
        const double s = std::sin(angleRad);
        return {x * c - y * s, x * s + y * c};
    }
};

constexpr Vec2 operator*(double s, const Vec2& v) { return v * s; }

}  // namespace interception
