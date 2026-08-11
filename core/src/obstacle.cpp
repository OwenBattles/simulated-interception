#include "interception/obstacle.hpp"

#include "interception/constants.hpp"

namespace interception {

Obstacle makeObstacle(Pcg32& rng, double worldWidthM, double worldHeightM) {
    Obstacle obstacle;
    // Sequenced deliberately: radius, then x, then y. See the note in
    // Actor's constructor -- C++ leaves argument evaluation order
    // unspecified, so brace-initialising from three rng calls would let the
    // compiler consume the stream in a different order than Python does.
    obstacle.radiusM =
        rng.uniform(constants::kMinObstacleRadiusM, constants::kMaxObstacleRadiusM);
    const double x = rng.uniform(obstacle.radiusM, worldWidthM - obstacle.radiusM);
    const double y = rng.uniform(obstacle.radiusM, worldHeightM - obstacle.radiusM);
    obstacle.pos = {x, y};
    return obstacle;
}

}  // namespace interception
