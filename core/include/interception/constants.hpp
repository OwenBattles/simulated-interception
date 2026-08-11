#pragma once

namespace interception::constants {

// UNITS: SI throughout -- metres, seconds, kilograms, newtons, radians.
// Nothing here is per-tick; every rate is per-second and is multiplied by
// dt at integration time. Pixels do not appear in the engine at all; the
// renderers own that conversion.
//
// Mirrors src/interception/constants.py. The two must stay in step, which
// the differential tests enforce.

// --- World --------------------------------------------------------------
inline constexpr double kWorldWidthM = 1500.0;
inline constexpr double kWorldHeightM = 1000.0;

// --- Simulation clock ---------------------------------------------------
inline constexpr double kSimDt = 1.0 / 60.0;
inline constexpr int kDefaultHeadlessMaxSteps = 10000;

// --- Interceptor --------------------------------------------------------
inline constexpr double kAgentMassKg = 5.0;
inline constexpr double kAgentMaxSpeedMps = 120.0;
inline constexpr double kAgentMaxForceN = 1000.0;  // 200 m/s^2 ~= 20 g
inline constexpr double kAgentHitRadiusM = 2.0;
inline constexpr double kAgentProbeLookaheadS = 0.8;

// --- Target -------------------------------------------------------------
inline constexpr double kTargetMassKg = 3.0;
inline constexpr double kTargetMaxSpeedMps = 80.0;
inline constexpr double kTargetMaxForceN = 300.0;  // 100 m/s^2 ~= 10 g
inline constexpr double kTargetHitRadiusM = 1.0;
inline constexpr double kTargetProbeLookaheadS = 0.8;

// Wander angle is a Wiener process, so its scale is rad per sqrt(second).
inline constexpr double kTargetWanderSigmaRadPerSqrtS = 0.6;
inline constexpr double kTargetWanderMaxRad = 1.2;
inline constexpr double kTargetWanderCircleDistM = 60.0;
inline constexpr double kTargetWanderCircleRadiusM = 60.0;

// --- Sensing ------------------------------------------------------------
inline constexpr double kProbeRadiusM = 25.0;
inline constexpr double kAvoidanceBrakingWeight = 0.2;

// --- Obstacles ----------------------------------------------------------
inline constexpr double kMinObstacleRadiusM = 30.0;
inline constexpr double kMaxObstacleRadiusM = 70.0;
inline constexpr int kMinObstacleCount = 5;
inline constexpr int kMaxObstacleCount = 10;

}  // namespace interception::constants
