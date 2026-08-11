#pragma once

#include "interception/params.hpp"
#include "interception/state.hpp"
#include "interception/vector.hpp"

namespace interception::test {

/// A world with no obstacles and effectively no walls, so unit tests
/// exercise one behaviour at a time instead of tripping over avoidance or
/// wall reflections.
inline ScenarioParams bareScenario(int agents = 1, int targets = 1) {
    ScenarioParams scenario;
    scenario.minObstacles = 0;
    scenario.maxObstacles = 0;
    scenario.numAgents = agents;
    scenario.numTargets = targets;
    scenario.worldWidthM = 1e7;
    scenario.worldHeightM = 1e7;
    return scenario;
}

/// Pin an actor's kinematics, bypassing its randomised spawn.
inline void place(Actor& actor, const Vec2& pos, const Vec2& vel) {
    actor.pos = pos;
    actor.vel = vel;
    actor.prevPos = pos;
    actor.reorient();
}

}  // namespace interception::test
