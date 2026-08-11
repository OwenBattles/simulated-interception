#include "interception/state.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "interception/collision.hpp"

namespace interception {

State::State(std::uint64_t seed, ScenarioParams scenario)
    : scenario_(std::move(scenario)), seed_(seed), rng_(seed) {
    buildWorld();
}

void State::reset() { reset(seed_); }

void State::reset(std::uint64_t seed) {
    seed_ = seed;
    rng_.reseed(seed_);
    buildWorld();
}

void State::buildWorld() {
    obstacles_.clear();
    targets_.clear();
    agents_.clear();

    // Construction order is part of the seed contract: obstacles, then
    // targets, then agents. Each pulls a fixed number of draws, so
    // reordering these loops changes every world.
    const int obstacleCount = rng_.randint(scenario_.minObstacles, scenario_.maxObstacles);
    obstacles_.reserve(static_cast<std::size_t>(obstacleCount));
    for (int i = 0; i < obstacleCount; ++i) {
        obstacles_.push_back(makeObstacle(rng_, widthM(), heightM()));
    }

    targets_.reserve(static_cast<std::size_t>(scenario_.numTargets));
    for (int i = 0; i < scenario_.numTargets; ++i) {
        targets_.emplace_back(*this, scenario_.target, rng_);
    }

    agents_.reserve(static_cast<std::size_t>(scenario_.numAgents));
    for (int i = 0; i < scenario_.numAgents; ++i) {
        agents_.emplace_back(*this, scenario_.interceptor, scenario_.guidance, rng_);
    }

    intercepts_ = 0;
    minMissDistanceM_ = std::numeric_limits<double>::infinity();
}

void State::update(double dt) {
    // Agents before targets, matching the reference implementation's
    // actor ordering. Obstacles are static and never step.
    for (Agent& agent : agents_) {
        agent.step(dt);
    }
    for (Target& target : targets_) {
        target.step(dt);
    }
    resolveIntercepts();
}

void State::resolveIntercepts() {
    if (targets_.empty()) {
        return;
    }

    std::vector<bool> doomed(targets_.size(), false);
    int doomedCount = 0;

    for (const Agent& agent : agents_) {
        for (std::size_t i = 0; i < targets_.size(); ++i) {
            const Target& target = targets_[i];
            const double combinedRadius = agent.hitRadiusM + target.hitRadiusM;
            const SweptHitResult result = sweptHit(agent.prevPos, agent.pos,
                                                   target.prevPos, target.pos,
                                                   combinedRadius);
            if (result.missDistanceM < minMissDistanceM_) {
                minMissDistanceM_ = result.missDistanceM;
            }
            if (result.hit && !doomed[i]) {
                doomed[i] = true;
                ++doomedCount;
            }
        }
    }

    if (doomedCount == 0) {
        return;
    }

    std::vector<Target> survivors;
    survivors.reserve(targets_.size() - static_cast<std::size_t>(doomedCount));
    for (std::size_t i = 0; i < targets_.size(); ++i) {
        if (!doomed[i]) {
            survivors.push_back(std::move(targets_[i]));
        }
    }
    targets_ = std::move(survivors);
    intercepts_ += doomedCount;
}

bool State::hasMinMissDistance() const { return std::isfinite(minMissDistanceM_); }

double State::totalDeltaVMps() const {
    double total = 0.0;
    for (const Agent& agent : agents_) {
        total += agent.deltaVMps;
    }
    return total;
}

}  // namespace interception
