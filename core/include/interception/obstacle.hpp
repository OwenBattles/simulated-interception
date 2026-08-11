#pragma once

#include "interception/rng.hpp"
#include "interception/vector.hpp"

namespace interception {

/// A static circular keep-out volume (building, no-fly zone, terrain).
///
/// Deliberately not an Actor: no mass, no velocity, never steps, so it
/// stays out of the update loop.
struct Obstacle {
    Vec2 pos;
    double radiusM = 0.0;
};

/// Draw a randomly placed obstacle.
///
/// The draw order -- radius, then x, then y -- is load-bearing. It must
/// match the Python reference exactly or the two engines generate different
/// worlds from the same seed.
Obstacle makeObstacle(Pcg32& rng, double worldWidthM, double worldHeightM);

}  // namespace interception
