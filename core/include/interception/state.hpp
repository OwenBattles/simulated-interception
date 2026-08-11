#pragma once

#include <cstdint>
#include <vector>

#include "interception/agent.hpp"
#include "interception/obstacle.hpp"
#include "interception/params.hpp"
#include "interception/rng.hpp"
#include "interception/target.hpp"

namespace interception {

/// The world: extent in metres, a seeded RNG, and every entity in it.
class State {
public:
    explicit State(std::uint64_t seed, ScenarioParams scenario = {});

    // Actors hold back-pointers to their world and to its RNG, so a State
    // must not be copied or moved once built -- those pointers would refer
    // to the moved-from object. Simulation owns States through a
    // unique_ptr for this reason.
    State(const State&) = delete;
    State& operator=(const State&) = delete;
    State(State&&) = delete;
    State& operator=(State&&) = delete;

    /// Rebuild from the stored seed. Never reseeds from entropy, so a
    /// reset is always repeatable.
    void reset();
    void reset(std::uint64_t seed);

    void update(double dt);

    double widthM() const { return scenario_.worldWidthM; }
    double heightM() const { return scenario_.worldHeightM; }
    std::uint64_t seed() const { return seed_; }
    const ScenarioParams& scenario() const { return scenario_; }
    Pcg32& rng() { return rng_; }

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    const std::vector<Agent>& agents() const { return agents_; }
    std::vector<Agent>& agents() { return agents_; }
    const std::vector<Target>& targets() const { return targets_; }
    std::vector<Target>& targets() { return targets_; }

    int intercepts() const { return intercepts_; }
    double minMissDistanceM() const { return minMissDistanceM_; }
    bool hasMinMissDistance() const;

    /// Summed control effort across the fleet.
    double totalDeltaVMps() const;

private:
    void buildWorld();

    /// Swept-sphere test over every agent/target pair for this step.
    /// Removals are deferred until after the scan: erasing mid-iteration
    /// would skip entries and invalidate the iterators doing the scanning.
    void resolveIntercepts();

    ScenarioParams scenario_;
    std::uint64_t seed_ = 0;
    Pcg32 rng_;
    std::vector<Obstacle> obstacles_;
    std::vector<Target> targets_;
    std::vector<Agent> agents_;
    int intercepts_ = 0;
    double minMissDistanceM_ = 0.0;
};

}  // namespace interception
