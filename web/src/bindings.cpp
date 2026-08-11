// WebAssembly bindings.
//
// The browser app drives the same core/ engine the native CLI and the
// Python extension use -- there is no second simulation implementation
// behind the web UI, and no server. The whole engagement runs client-side,
// which is what lets the app deploy as a static page.
//
// The surface is deliberately small: configure, reset, step, snapshot.
// Everything tunable in the UI maps onto a field of ScenarioParams,
// VehicleParams, or GuidanceParams.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <cstdint>
#include <memory>
#include <string>

#include "interception/simulation.hpp"
#include "interception/telemetry.hpp"

using emscripten::val;
using namespace interception;

namespace {

/// Read `key` from a JS object, falling back when absent. Lets the UI send
/// partial updates instead of the entire parameter tree every time.
double number(const val& source, const char* key, double fallback) {
    if (source.isUndefined() || source.isNull()) {
        return fallback;
    }
    const val value = source[key];
    if (value.isUndefined() || value.isNull()) {
        return fallback;
    }
    return value.as<double>();
}

std::string text(const val& source, const char* key, const std::string& fallback) {
    if (source.isUndefined() || source.isNull()) {
        return fallback;
    }
    const val value = source[key];
    if (value.isUndefined() || value.isNull()) {
        return fallback;
    }
    return value.as<std::string>();
}

VehicleParams readVehicle(const val& source, VehicleParams defaults) {
    defaults.massKg = number(source, "massKg", defaults.massKg);
    defaults.maxSpeedMps = number(source, "maxSpeedMps", defaults.maxSpeedMps);
    defaults.maxForceN = number(source, "maxForceN", defaults.maxForceN);
    defaults.hitRadiusM = number(source, "hitRadiusM", defaults.hitRadiusM);
    defaults.probeLookaheadS =
        number(source, "probeLookaheadS", defaults.probeLookaheadS);
    defaults.probeRadiusM = number(source, "probeRadiusM", defaults.probeRadiusM);
    return defaults;
}

val vehicleToVal(const VehicleParams& params) {
    val out = val::object();
    out.set("massKg", params.massKg);
    out.set("maxSpeedMps", params.maxSpeedMps);
    out.set("maxForceN", params.maxForceN);
    out.set("hitRadiusM", params.hitRadiusM);
    out.set("probeLookaheadS", params.probeLookaheadS);
    out.set("probeRadiusM", params.probeRadiusM);
    out.set("maxAccelMps2", params.maxAccelMps2());
    out.set("maxAccelG", params.maxAccelG());
    out.set("turnRadiusM", params.turnRadiusM());
    return out;
}

val actorToVal(const Actor& actor) {
    val out = val::object();
    out.set("x", actor.pos.x);
    out.set("y", actor.pos.y);
    out.set("vx", actor.vel.x);
    out.set("vy", actor.vel.y);
    out.set("heading", actor.forwardVec.angle());
    out.set("speedMps", actor.vel.magnitude());
    out.set("probeX", actor.probe.pos.x);
    out.set("probeY", actor.probe.pos.y);
    out.set("probeRadiusM", actor.probe.radiusM);
    return out;
}

}  // namespace

/// Thin lifetime wrapper. Simulation is not copyable, and embind wants a
/// class it can own, so the browser holds one of these for the page's life
/// and reconfigures it in place.
class WebSimulation {
public:
    WebSimulation() { rebuild(); }

    /// Apply a (possibly partial) parameter object and rebuild the world.
    void configure(const val& options) {
        scenario_.worldWidthM = number(options, "worldWidthM", scenario_.worldWidthM);
        scenario_.worldHeightM =
            number(options, "worldHeightM", scenario_.worldHeightM);
        scenario_.numAgents =
            static_cast<int>(number(options, "numAgents", scenario_.numAgents));
        scenario_.numTargets =
            static_cast<int>(number(options, "numTargets", scenario_.numTargets));
        scenario_.minObstacles =
            static_cast<int>(number(options, "minObstacles", scenario_.minObstacles));
        scenario_.maxObstacles =
            static_cast<int>(number(options, "maxObstacles", scenario_.maxObstacles));

        scenario_.interceptor = readVehicle(options["interceptor"], scenario_.interceptor);
        scenario_.target = readVehicle(options["target"], scenario_.target);

        const val guidance = options["guidance"];
        scenario_.guidance.law = guidanceLawFromString(
            text(guidance, "law", guidanceLawName(scenario_.guidance.law)));
        scenario_.guidance.navConstant =
            number(guidance, "navConstant", scenario_.guidance.navConstant);

        dt_ = number(options, "dt", dt_);
        if (!options.isUndefined() && !options["seed"].isUndefined() &&
            !options["seed"].isNull()) {
            seed_ = static_cast<std::uint64_t>(number(options, "seed", 0));
        }
        rebuild();
    }

    /// Rebuild with the stored seed, or a specific one.
    void reset() { rebuild(); }

    void reseed(double seed) {
        seed_ = static_cast<std::uint64_t>(seed);
        rebuild();
    }

    void step() { sim_->step(); }

    /// Advance several ticks per animation frame, so the sim clock can run
    /// faster than the display refresh without changing the physics dt.
    void stepMany(int count) {
        for (int i = 0; i < count && !sim_->done(); ++i) {
            sim_->step();
        }
    }

    bool done() const { return sim_->done(); }

    /// Everything the renderer and the readout need for one frame.
    val snapshot() const {
        const State& state = sim_->state();
        const Observation obs = sim_->observation();

        val agents = val::array();
        int index = 0;
        for (const Agent& agent : state.agents()) {
            val entry = actorToVal(agent);
            entry.set("deltaVMps", agent.deltaVMps);
            entry.set("accelMps2", agent.acc.magnitude());
            const GuidanceDiagnostics diagnostics = agent.diagnostics();
            if (agent.currentTarget() != nullptr) {
                entry.set("rangeM", diagnostics.rangeM);
                entry.set("closingSpeedMps", diagnostics.closingSpeedMps);
                entry.set("losRateRadS", diagnostics.losRateRadS);
            }
            agents.set(index++, entry);
        }

        val targets = val::array();
        index = 0;
        for (const Target& target : state.targets()) {
            targets.set(index++, actorToVal(target));
        }

        val obstacles = val::array();
        index = 0;
        for (const Obstacle& obstacle : state.obstacles()) {
            val entry = val::object();
            entry.set("x", obstacle.pos.x);
            entry.set("y", obstacle.pos.y);
            entry.set("radiusM", obstacle.radiusM);
            obstacles.set(index++, entry);
        }

        val out = val::object();
        out.set("seed", static_cast<double>(obs.seed));
        out.set("guidance", obs.guidance);
        out.set("step", obs.step);
        out.set("elapsedS", obs.elapsedS);
        out.set("done", obs.done);
        out.set("endReason", obs.endReason);
        out.set("intercepts", obs.intercepts);
        out.set("minMissDistanceM",
                obs.hasMinMissDistance ? val(obs.minMissDistanceM) : val::null());
        out.set("deltaVMps", obs.deltaVMps);
        out.set("worldWidthM", state.widthM());
        out.set("worldHeightM", state.heightM());
        out.set("agents", agents);
        out.set("targets", targets);
        out.set("obstacles", obstacles);
        return out;
    }

    /// The full current parameter tree, so the UI can seed its controls
    /// from the engine's defaults rather than duplicating them in JS.
    val params() const {
        val guidance = val::object();
        guidance.set("law", guidanceLawName(scenario_.guidance.law));
        guidance.set("navConstant", scenario_.guidance.navConstant);

        val out = val::object();
        out.set("worldWidthM", scenario_.worldWidthM);
        out.set("worldHeightM", scenario_.worldHeightM);
        out.set("numAgents", scenario_.numAgents);
        out.set("numTargets", scenario_.numTargets);
        out.set("minObstacles", scenario_.minObstacles);
        out.set("maxObstacles", scenario_.maxObstacles);
        out.set("interceptor", vehicleToVal(scenario_.interceptor));
        out.set("target", vehicleToVal(scenario_.target));
        out.set("guidance", guidance);
        out.set("dt", dt_);
        out.set("seed", static_cast<double>(seed_));
        return out;
    }

private:
    void rebuild() {
        SimulationConfig config;
        config.dt = dt_;
        config.maxSteps = 0;  // interactive runs never time out
        config.hasSeed = true;
        config.seed = seed_;
        config.scenario = scenario_;
        sim_ = std::make_unique<Simulation>(std::move(config));
    }

    ScenarioParams scenario_{};
    double dt_ = constants::kSimDt;
    std::uint64_t seed_ = 0;
    std::unique_ptr<Simulation> sim_;
};

EMSCRIPTEN_BINDINGS(interception) {
    emscripten::class_<WebSimulation>("Simulation")
        .constructor<>()
        .function("configure", &WebSimulation::configure)
        .function("reset", &WebSimulation::reset)
        .function("reseed", &WebSimulation::reseed)
        .function("step", &WebSimulation::step)
        .function("stepMany", &WebSimulation::stepMany)
        .function("done", &WebSimulation::done)
        .function("snapshot", &WebSimulation::snapshot)
        .function("params", &WebSimulation::params);
}
