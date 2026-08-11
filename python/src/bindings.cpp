// pybind11 bindings.
//
// Exposes the C++ engine to Python so the CLI, the plotting scripts, and
// the pygame view keep working against a single simulation implementation.
//
// Names are snake_case on purpose: this module replaces a pure-Python
// engine, and matching its API keeps the call sites unchanged.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "interception/simulation.hpp"
#include "interception/telemetry.hpp"

namespace py = pybind11;
using namespace interception;

namespace {

py::dict observationToDict(const Observation& obs) {
    py::dict out;
    out["seed"] = obs.seed;
    out["guidance"] = obs.guidance;
    out["step"] = obs.step;
    // Rounded for JSON friendliness and stable golden fixtures; the
    // unrounded value is available through Simulation.elapsed_s.
    out["elapsed_s"] = std::round(obs.elapsedS * 1e4) / 1e4;
    out["done"] = obs.done;
    out["end_reason"] = obs.endReason;
    out["intercepts"] = obs.intercepts;
    if (obs.hasMinMissDistance) {
        out["min_miss_distance_m"] = std::round(obs.minMissDistanceM * 1e4) / 1e4;
    } else {
        out["min_miss_distance_m"] = py::none();
    }
    out["delta_v_mps"] = std::round(obs.deltaVMps * 1e3) / 1e3;
    out["num_targets"] = obs.numTargets;
    out["num_agents"] = obs.numAgents;
    out["num_obstacles"] = obs.numObstacles;
    return out;
}

py::dict actorFrameToDict(const ActorFrame& frame) {
    py::dict out;
    out["x"] = frame.x;
    out["y"] = frame.y;
    out["vx"] = frame.vx;
    out["vy"] = frame.vy;
    out["speed_mps"] = frame.speedMps;
    return out;
}

py::dict agentFrameToDict(const AgentFrame& frame) {
    py::dict out = actorFrameToDict(frame);
    out["delta_v_mps"] = frame.deltaVMps;
    out["accel_mps2"] = frame.accelMps2;
    if (frame.hasGuidance) {
        out["range_m"] = frame.guidance.rangeM;
        out["closing_speed_mps"] = frame.guidance.closingSpeedMps;
        out["los_rate_rad_s"] = frame.guidance.losRateRadS;
        if (frame.guidance.hasLateralAccel) {
            out["lateral_accel_mps2"] = frame.guidance.lateralAccelMps2;
        }
    }
    return out;
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "C++ interception engine exposed to Python.";

    // --- geometry -------------------------------------------------------
    py::class_<Vec2>(m, "Vector")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("x") = 0.0, py::arg("y") = 0.0)
        .def_readwrite("x", &Vec2::x)
        .def_readwrite("y", &Vec2::y)
        .def("pair", [](const Vec2& v) { return py::make_tuple(v.x, v.y); })
        .def("magnitude", &Vec2::magnitude)
        .def("dist_to", &Vec2::distTo)
        .def("angle", &Vec2::angle)
        .def("__repr__", [](const Vec2& v) {
            return "Vector(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        });

    py::class_<Probe>(m, "Probe")
        .def_readonly("lookahead_s", &Probe::lookaheadS)
        .def_readonly("radius_m", &Probe::radiusM)
        .def_readonly("pos", &Probe::pos);

    py::class_<Obstacle>(m, "Obstacle")
        .def_readonly("pos", &Obstacle::pos)
        .def_readonly("radius_m", &Obstacle::radiusM);

    // --- parameters -----------------------------------------------------
    py::class_<VehicleParams>(m, "VehicleParams")
        .def(py::init([](double mass_kg, double max_speed_mps, double max_force_n,
                         double hit_radius_m, double probe_lookahead_s,
                         double probe_radius_m) {
                 VehicleParams p;
                 p.massKg = mass_kg;
                 p.maxSpeedMps = max_speed_mps;
                 p.maxForceN = max_force_n;
                 p.hitRadiusM = hit_radius_m;
                 p.probeLookaheadS = probe_lookahead_s;
                 p.probeRadiusM = probe_radius_m;
                 return p;
             }),
             py::arg("mass_kg") = constants::kAgentMassKg,
             py::arg("max_speed_mps") = constants::kAgentMaxSpeedMps,
             py::arg("max_force_n") = constants::kAgentMaxForceN,
             py::arg("hit_radius_m") = constants::kAgentHitRadiusM,
             py::arg("probe_lookahead_s") = constants::kAgentProbeLookaheadS,
             py::arg("probe_radius_m") = constants::kProbeRadiusM)
        .def_readwrite("mass_kg", &VehicleParams::massKg)
        .def_readwrite("max_speed_mps", &VehicleParams::maxSpeedMps)
        .def_readwrite("max_force_n", &VehicleParams::maxForceN)
        .def_readwrite("hit_radius_m", &VehicleParams::hitRadiusM)
        .def_readwrite("probe_lookahead_s", &VehicleParams::probeLookaheadS)
        .def_readwrite("probe_radius_m", &VehicleParams::probeRadiusM)
        .def_property_readonly("max_accel_mps2", &VehicleParams::maxAccelMps2)
        .def_property_readonly("max_accel_g", &VehicleParams::maxAccelG)
        .def_property_readonly("turn_radius_m", &VehicleParams::turnRadiusM);

    m.def("default_interceptor", &defaultInterceptor);
    m.def("default_target", &defaultTarget);

    py::class_<GuidanceParams>(m, "GuidanceParams")
        .def(py::init([](const std::string& law, double nav_constant) {
                 GuidanceParams p;
                 p.law = guidanceLawFromString(law);
                 p.navConstant = nav_constant;
                 return p;
             }),
             py::arg("law") = "pn", py::arg("nav_constant") = 4.0)
        .def_property(
            "law", [](const GuidanceParams& p) { return guidanceLawName(p.law); },
            [](GuidanceParams& p, const std::string& law) {
                p.law = guidanceLawFromString(law);
            })
        .def_readwrite("nav_constant", &GuidanceParams::navConstant);

    py::class_<ScenarioParams>(m, "ScenarioParams")
        .def(py::init([](double world_width_m, double world_height_m, int num_agents,
                         int num_targets, int min_obstacles, int max_obstacles,
                         std::optional<VehicleParams> interceptor,
                         std::optional<VehicleParams> target,
                         std::optional<GuidanceParams> guidance) {
                 ScenarioParams s;
                 s.worldWidthM = world_width_m;
                 s.worldHeightM = world_height_m;
                 s.numAgents = num_agents;
                 s.numTargets = num_targets;
                 s.minObstacles = min_obstacles;
                 s.maxObstacles = max_obstacles;
                 if (interceptor) s.interceptor = *interceptor;
                 if (target) s.target = *target;
                 if (guidance) s.guidance = *guidance;
                 return s;
             }),
             py::arg("world_width_m") = constants::kWorldWidthM,
             py::arg("world_height_m") = constants::kWorldHeightM,
             py::arg("num_agents") = 1, py::arg("num_targets") = 1,
             py::arg("min_obstacles") = constants::kMinObstacleCount,
             py::arg("max_obstacles") = constants::kMaxObstacleCount,
             py::arg("interceptor") = py::none(), py::arg("target") = py::none(),
             py::arg("guidance") = py::none())
        .def_readwrite("world_width_m", &ScenarioParams::worldWidthM)
        .def_readwrite("world_height_m", &ScenarioParams::worldHeightM)
        .def_readwrite("num_agents", &ScenarioParams::numAgents)
        .def_readwrite("num_targets", &ScenarioParams::numTargets)
        .def_readwrite("min_obstacles", &ScenarioParams::minObstacles)
        .def_readwrite("max_obstacles", &ScenarioParams::maxObstacles)
        .def_readwrite("interceptor", &ScenarioParams::interceptor)
        .def_readwrite("target", &ScenarioParams::target)
        .def_readwrite("guidance", &ScenarioParams::guidance);

    // --- actors ---------------------------------------------------------
    // Read-only views. These borrow from the State that owns them, so a
    // reset or an intercept invalidates any handle held across it -- the
    // renderer re-reads every frame rather than caching.
    py::class_<Actor>(m, "Actor")
        .def_readonly("pos", &Actor::pos)
        .def_readonly("vel", &Actor::vel)
        .def_readonly("acc", &Actor::acc)
        .def_readonly("forward_vec", &Actor::forwardVec)
        .def_readonly("side_vec", &Actor::sideVec)
        .def_readonly("probe", &Actor::probe)
        .def_readonly("hit_radius_m", &Actor::hitRadiusM)
        .def_readonly("delta_v_mps", &Actor::deltaVMps)
        .def_readonly("max_speed", &Actor::maxSpeed)
        .def_readonly("max_force", &Actor::maxForce)
        .def_readonly("mass", &Actor::mass);

    py::class_<Agent, Actor>(m, "Agent")
        .def_property_readonly(
            "guidance_law",
            [](const Agent& a) { return a.guidance().name(); });

    py::class_<Target, Actor>(m, "Target")
        .def_readonly("wander_angle", &Target::wanderAngle);

    // --- world ----------------------------------------------------------
    py::class_<State>(m, "State")
        .def_property_readonly("width", &State::widthM)
        .def_property_readonly("height", &State::heightM)
        .def_property_readonly("seed", &State::seed)
        .def_property_readonly("intercepts", &State::intercepts)
        .def_property_readonly(
            "min_miss_distance_m",
            [](const State& s) {
                return s.hasMinMissDistance() ? py::cast(s.minMissDistanceM())
                                              : py::cast(std::nullopt);
            })
        .def_property_readonly(
            "agents",
            [](State& s) {
                std::vector<Agent*> out;
                out.reserve(s.agents().size());
                for (Agent& a : s.agents()) out.push_back(&a);
                return out;
            },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "targets",
            [](State& s) {
                std::vector<Target*> out;
                out.reserve(s.targets().size());
                for (Target& t : s.targets()) out.push_back(&t);
                return out;
            },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "obstacles",
            [](const State& s) {
                std::vector<const Obstacle*> out;
                out.reserve(s.obstacles().size());
                for (const Obstacle& o : s.obstacles()) out.push_back(&o);
                return out;
            },
            py::return_value_policy::reference_internal);

    // --- simulation -----------------------------------------------------
    py::class_<SimulationConfig>(m, "SimulationConfig")
        .def(py::init([](double dt, int max_steps, std::optional<std::uint64_t> seed,
                         std::optional<ScenarioParams> scenario,
                         bool record_telemetry) {
                 SimulationConfig c;
                 c.dt = dt;
                 c.maxSteps = max_steps;
                 c.hasSeed = seed.has_value();
                 c.seed = seed.value_or(0);
                 if (scenario) c.scenario = *scenario;
                 c.recordTelemetry = record_telemetry;
                 return c;
             }),
             py::arg("dt") = constants::kSimDt, py::arg("max_steps") = 0,
             py::arg("seed") = py::none(), py::arg("scenario") = py::none(),
             py::arg("record_telemetry") = false);

    py::class_<TelemetryRecorder>(m, "TelemetryRecorder")
        .def_property_readonly("frames", [](const TelemetryRecorder& recorder) {
            py::list frames;
            for (const TelemetryFrame& frame : recorder.frames()) {
                py::dict entry;
                entry["t"] = std::round(frame.t * 1e4) / 1e4;
                entry["step"] = frame.step;
                py::list agents;
                for (const AgentFrame& agent : frame.agents) {
                    agents.append(agentFrameToDict(agent));
                }
                py::list targets;
                for (const ActorFrame& target : frame.targets) {
                    targets.append(actorFrameToDict(target));
                }
                entry["agents"] = agents;
                entry["targets"] = targets;
                frames.append(entry);
            }
            return frames;
        });

    py::class_<Simulation>(m, "Simulation")
        .def(py::init([](std::optional<SimulationConfig> config) {
                 return std::make_unique<Simulation>(config.value_or(SimulationConfig{}));
             }),
             py::arg("config") = py::none())
        .def("reset",
             [](Simulation& sim, std::optional<std::uint64_t> seed) {
                 if (seed) {
                     sim.reset(*seed);
                 } else {
                     sim.reset();
                 }
             },
             py::arg("seed") = py::none())
        .def("step", &Simulation::step)
        .def("run", &Simulation::run, py::return_value_policy::reference_internal)
        .def("observation", [](const Simulation& sim) {
            return observationToDict(sim.observation());
        })
        // state() and telemetry() are const/non-const overload pairs, so the
        // member-pointer form is ambiguous; pick the mutable one explicitly.
        .def_property_readonly(
            "state", [](Simulation& sim) -> State& { return sim.state(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "telemetry",
            [](Simulation& sim) -> TelemetryRecorder* { return sim.telemetry(); },
            py::return_value_policy::reference_internal)
        .def_property_readonly("seed", &Simulation::seed)
        .def_property_readonly("steps", &Simulation::steps)
        .def_property_readonly("dt", &Simulation::dt)
        .def_property_readonly("elapsed_s", &Simulation::elapsedS)
        .def_property_readonly("done", &Simulation::done)
        .def_property_readonly("end_reason", [](const Simulation& sim) {
            return episodeEndName(sim.endReason());
        });

    m.def(
        "run_headless",
        [](std::optional<std::uint64_t> seed, int max_steps,
           std::optional<ScenarioParams> scenario, bool record_telemetry) {
            SimulationConfig config;
            config.hasSeed = seed.has_value();
            config.seed = seed.value_or(0);
            config.maxSteps = max_steps;
            if (scenario) config.scenario = *scenario;
            config.recordTelemetry = record_telemetry;
            auto sim = std::make_unique<Simulation>(std::move(config));
            sim->run();
            return sim;
        },
        py::arg("seed") = py::none(),
        py::arg("max_steps") = constants::kDefaultHeadlessMaxSteps,
        py::arg("scenario") = py::none(), py::arg("record_telemetry") = false);

    m.attr("SIM_DT") = constants::kSimDt;
    m.attr("DEFAULT_HEADLESS_MAX_STEPS") = constants::kDefaultHeadlessMaxSteps;
    m.attr("WORLD_WIDTH_M") = constants::kWorldWidthM;
    m.attr("WORLD_HEIGHT_M") = constants::kWorldHeightM;
    m.attr("GUIDANCE_LAWS") = py::make_tuple("apn", "lead", "pn", "pursuit");
}
