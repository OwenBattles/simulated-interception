# Modelling assumptions

Every simplification the simulator makes, and what it would take to lift it.
This exists so results are read with the right caveats rather than taken as
flight-representative.

## Kinematics

| Assumption | Detail | Cost of lifting |
| --- | --- | --- |
| 2D, flat world | Planar motion in a 1500 m x 1000 m box. No altitude, no terrain masking. | Moderate: the vector maths generalises, but guidance and avoidance both gain a plane-selection problem. |
| Point-mass dynamics | Vehicles are point masses steered by a force. No attitude state, no moments of inertia, no aerodynamic angles. | Large: a 6DOF rewrite with a rate-limited autopilot inner loop. |
| Velocity-aligned body frame | Heading is defined as the velocity direction, so there is no sideslip or angle of attack. | Follows from the point-mass model. |
| Instant force response | Commanded steering force is applied the same tick. Real actuators have bandwidth and rate limits. | Small: a first-order lag on the command. |
| No drag, no gravity, no wind | Speed is bounded by an explicit `max_speed` cap rather than a thrust/drag balance. | Small for drag; wind needs an environment model. |
| Constant mass | No propellant burn, so `delta-v` is a control-effort proxy rather than a mass budget. | Small. |

## Guidance and sensing

| Assumption | Detail | Cost of lifting |
| --- | --- | --- |
| Perfect state knowledge | The interceptor reads exact target position and velocity from world state. No noise, no latency, no dropouts. | This is the single largest gap. Needs a measurement model plus an estimator (EKF) on the interceptor. |
| Unlimited detection range | Every target is visible from anywhere in the box. No seeker field-of-view or acquisition range. | Small: a range/FOV gate on `current_target`. |
| First-order lead pursuit | Time-to-go is estimated as `range / interceptor max speed`, which is exact head-on and degrades in a crossing geometry. | Small, and it is the next planned change: proportional navigation. |
| Nearest-target selection | Each interceptor independently chases the closest target, so several can converge on the same one. | Small: a fleet-level assignment step. |
| Non-adversarial evader | The target flies a random-walk wander and never reacts to being chased. | Small in code, large in what it changes: this is why success rate currently sits at 100%. |
| Avoidance overrides pursuit | Any obstacle in probe range fully replaces the pursuit command rather than blending with it. | Small: weighted blending or a velocity-obstacle formulation. |

## World

| Assumption | Detail | Cost of lifting |
| --- | --- | --- |
| Circular obstacles | Keep-out volumes are discs. Only the probe collides with them; vehicles are not destroyed by contact. | Small for polygons; scoring obstacle strikes is a rules change. |
| Reflecting walls | Vehicles bounce off the box edge. Physically arbitrary -- it exists to keep the engagement bounded. | Small: replace with an out-of-bounds termination condition. |
| Obstacles may overlap spawns | Placement is uniform-random and does not deconflict against vehicle start positions. | Small: rejection sampling. |

## Numerics

- **Integrator**: semi-implicit (symplectic) Euler at a fixed 1/60 s timestep.
  Velocity is exact under constant force; position carries `O(dt)` truncation
  error. `tests/test_integration.py` pins both properties, including that the
  position error shrinks with the timestep.
- **Collision**: swept-sphere closest-approach over each step, not endpoint
  sampling. At a 200 m/s closing rate the pair advances 3.3 m per tick against
  a 3 m capture radius, so endpoint tests tunnel.
- **Randomness**: one seeded `random.Random` per world. Runs without an
  explicit seed draw one from `SystemRandom`, record it, and report it, so
  every episode is replayable after the fact.
- **Wander process**: the evader's wander angle is a Wiener process with
  increments scaled by `sqrt(dt)`, so its path statistics do not change with
  the timestep.
