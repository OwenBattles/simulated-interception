# Interception simulator

A 2D counter-UAS engagement simulator: an interceptor runs down a manoeuvring
target through a field of keep-out volumes, flying a guidance law you can swap
at runtime, and the run is scored on the metrics an engagement is actually
judged by — probability of kill, time to intercept, miss distance, and control
effort.

The core is dependency-free Python that runs headless and deterministically;
pygame is optional and only needed to open a window.

![Interceptor trajectory against a wandering evader](results/figures/engagement.svg)

## Scenario

A fixed-wing interceptor engages a quadrotor inside a 1500 m × 1000 m box
containing 5–10 circular no-fly volumes.

|  | Interceptor | Target |
| --- | --- | --- |
| Max speed | 120 m/s | 80 m/s |
| Max lateral acceleration | 200 m/s² (≈20 g) | 100 m/s² (≈10 g) |
| Turn radius at max speed | 72 m | 64 m |
| Mass | 5 kg | 3 kg |
| Capture radius | 2 m | 1 m |

The target turns tighter than the interceptor, which is what makes the
geometry non-trivial: raw speed advantage does not guarantee an intercept.

## Guidance

Four laws share one interface, so they can be swapped at runtime and compared
on identical seeds.

| Law | Command | Idea |
| --- | --- | --- |
| `pursuit` | point at where the target **is** | The naive baseline. Always converts into a tail chase. |
| `lead` | point at where the target **will be**, using `t_go = range / max speed` | Exact head-on, degrades in a crossing geometry. |
| `pn` | `a = N · V_c · λ̇`, normal to the line of sight | Drives the LOS rate to zero. |
| `apn` | PN plus `N/2` of the target's LOS-normal acceleration | Recovers the lag PN shows against a manoeuvring target. |

The insight behind PN is that a collision course is *exactly* the condition
`λ̇ = 0`: if the bearing to the target is not rotating, the two are converging
on the same point. So rather than predicting where the target will be, PN just
nulls the rotation of the sight line — no prediction required. That is what
real interceptors fly.

Guidance commands turn only; whatever force budget the turn leaves over is
spent closing to top speed, with turning given priority.

![LOS rate against time for each guidance law](results/figures/guidance-comparison.svg)

Against a steady 3 g turning target the textbook ordering comes out cleanly.
Pure pursuit lets the LOS rate run away as range closes; PN holds it near zero
and intercepts 1.5 s sooner for half the control effort; APN nulls the residual
that PN leaves against an accelerating target and does it for **8× less Δv than
pursuit**.

## Results

500 seeds per law, 1 interceptor vs 1 target, `interception --trials 500 --guidance <law>`:

| Guidance | Success | Mean TTI | Median TTI | Mean miss | Mean Δv |
| --- | --- | --- | --- | --- | --- |
| `pursuit` | 100% | 9.01 s | 8.30 s | 2.52 m | 938 m/s |
| `lead` | 100% | 8.16 s | 7.31 s | 2.29 m | 984 m/s |
| `pn` | 100% | 10.53 s | 8.92 s | **2.01 m** | 958 m/s |
| `apn` | 100% | 10.92 s | 9.71 s | **1.85 m** | 1467 m/s |

**This is not the ordering the comparison figure shows, and that is the
interesting part.** Miss distance improves monotonically from pursuit to APN,
but PN is 29% *slower* to intercept than lead pursuit here, and APN spends 50%
more Δv than anything else.

The cause is the target, not the law. This evader flies a random walk, so its
velocity is essentially noise:

- PN's premise is that both vehicles are on steady courses, which makes nulling
  `λ̇` equivalent to a collision course. Against a target that re-randomises its
  heading continuously, the collision course PN establishes is invalidated
  faster than PN converges on it, while pursuit and lead simply keep pointing at
  a target that is never far off the nose.
- APN feeds forward the target's instantaneous lateral acceleration. For a
  random walk that quantity *is* the noise, so APN amplifies it — measured mean
  commanded acceleration rises to 135 m/s² against PN's 91 m/s².

It is not a control-authority problem: PN commands 23% less mean acceleration
than lead and saturates the airframe half as often (24% of ticks vs 43%), while
flying 4% faster on average. It simply is not the right law for an incoherent
target.

The honest conclusion is that **the scenario is now the limiting factor, not the
guidance**. A random-walk evader cannot distinguish these laws the way a
coherent manoeuvre does — which is exactly why an adversarial evader is the next
item on the roadmap.

Fleet configurations, PN, 500 seeds each:

| Configuration | Success | Mean TTI | Median TTI | Mean miss | Mean Δv |
| --- | --- | --- | --- | --- | --- |
| 1 interceptor, 1 target | 100% | 10.53 s | 8.92 s | 2.01 m | 958 m/s |
| 1 interceptor, 3 targets | 100% | 26.53 s | 24.57 s | 1.33 m | 2642 m/s |
| 3 interceptors, 3 targets | 100% | 12.43 s | 11.48 s | 1.29 m | 3380 m/s |

Δv is summed across the fleet, which is why it rises with interceptor count
even as time-to-intercept falls.

**Read the 100% honestly.** Success rate currently carries no information — the
evader never reacts to being chased, so a 1.5× speed advantage is decisive under
every law. Miss distance and Δv are the metrics doing real work today.

## Quickstart

```bash
pip install -e .              # core only, no dependencies
interception --headless --seed 0
interception --headless --guidance apn --nav-constant 4.5
interception --trials 500 --guidance pn        # seed sweep with aggregates
interception --record runs/engagement.json     # per-step telemetry
interception --trials 100 --json > runs.json

pip install -e '.[viz]'       # adds pygame
interception                  # opens a window
```

Interactive keys: `space` pause · `n` single-step · `r` reset · `p` toggle
probes · `esc` quit.

Regenerate the figures:

```bash
python scripts/plot_engagement.py --seed 0
python scripts/plot_guidance_comparison.py
```

## Design notes

**SI units, everywhere.** Every quantity in the core is metres, seconds,
kilograms, and newtons. Nothing is expressed per-tick: rates are per-second and
multiplied by `dt` at integration time, so trajectories are identical at 60 Hz
and 240 Hz. Pixels exist only in `render.py`. This is enforced by tests, not
convention — `tests/test_integration.py` runs the same manoeuvre at three
timesteps and asserts the results agree.

**Parameters are objects, not module globals.** Airframe limits, guidance
tuning, and scenario layout live in dataclasses (`params.py`) that are passed
through a run. `constants.py` supplies only the defaults. Nothing needs a
module reload to change a vehicle's g-limit or the navigation constant, which
is what makes a live-tunable dashboard possible.

**The core never imports pygame.** Rendering is a separate module that reads
world state from outside; entities know nothing about drawing. CI installs the
base package with no pygame at all, and a test asserts the import graph stays
clean, so headless runs can never quietly acquire an SDL dependency.

**Every run is replayable.** One seeded RNG per world. A run started without a
seed draws one from `SystemRandom`, stores it, and reports it in the output —
so an interesting unseeded episode can always be reproduced exactly. `reset()`
replays the recorded seed rather than reseeding from entropy.

**Continuous collision detection.** Interception is resolved by a swept-sphere
closest-approach test over each timestep, not by sampling positions. At a
200 m/s closing rate the pair advances 3.3 m per tick against a 3 m capture
radius, so endpoint sampling passes straight through the target. The same test
yields miss distance for free.

**Telemetry is opt-in and non-perturbing.** `--record` captures a per-step
frame log — position, speed, commanded acceleration, range, closing speed, LOS
rate — without changing the trajectory it observes, which a test asserts.

Modelling simplifications are catalogued in [docs/assumptions.md](docs/assumptions.md).

## Layout

```
src/interception/
  vector.py       2D vector maths
  constants.py    default parameter values, SI
  params.py       runtime-tunable parameter objects
  actor.py        integration, orientation, bounds, obstacle avoidance
  guidance.py     pursuit / lead / PN / APN, and force allocation
  agent.py        interceptor: guidance + avoidance arbitration
  target.py       evader (Reynolds wander)
  obstacle.py     static keep-out volumes
  sensor.py       forward-looking collision probe
  collision.py    swept-sphere closest approach
  state.py        world, entity lists, intercept resolution
  simulation.py   episode lifecycle, config, metrics
  telemetry.py    per-step frame log
  fleet.py        interceptor group
  render.py       all pygame drawing
  view.py         window, input, frame loop
  cli.py          entrypoint
scripts/          reproducible figure generation
tests/            64 tests, no pygame required
docs/             modelling assumptions
```

## Testing

```bash
pip install -e '.[dev]'
pytest
```

64 tests covering vector maths, timestep independence, force and speed
clamping, swept-collision geometry, LOS geometry, guidance-law behaviour,
obstacle avoidance, world determinism, telemetry, and episode lifecycle. CI
runs the suite on Python 3.10–3.12 plus a headless smoke run.

The guidance tests pin *behaviour*, not just execution: that PN drives LOS rate
toward zero and pure pursuit lets it grow, that PN's terminal course is
straight, that PN spends less control effort than pursuit, and that APN reduces
exactly to PN when the target stops accelerating.

Three further tests are regressions for bugs found in earlier revisions, written
to fail if the fix is reverted:

- Comparing distances to obstacles passed an `Obstacle` where a `Vector` was
  expected. A short-circuiting `or` hid it until a *second* obstacle entered
  probe range, so it only surfaced in cluttered geometry.
- Intercepts mutated the actor list while the step loop was iterating it,
  silently skipping actors on any tick where a target died.
- `reset()` reseeded from OS entropy when called without an argument, quietly
  breaking the reproducibility guarantee.

## Roadmap

Ordered by how much each changes what the simulator can tell you.

1. ~~**Proportional navigation.**~~ Done — PN and APN alongside pursuit and lead,
   benchmarked against each other.
2. **Adversarial evader.** Coherent evasive manoeuvres — weave, break turn, jink
   on detection — with a difficulty parameter, replacing the random walk. This
   is the blocker on everything downstream: until the target manoeuvres
   coherently, probability of kill is pinned at 100% and the guidance laws
   cannot be told apart on the live scenario.
3. **Monte Carlo harness.** Sweep difficulty × guidance law × navigation
   constant across thousands of seeded runs; publish Pk curves, miss-distance
   CDFs, and Δv budgets.

### Dashboard

The telemetry recorder and parameter objects exist to feed this; both are
already in place.

4. **Static run report.** A self-contained HTML page generated from a telemetry
   JSON — trajectory, LOS-rate/acceleration/range traces, and the episode
   summary, with no server and no dependencies. Publishable to GitHub Pages, so
   an engagement can be shared as a permanent link rather than a screenshot.
5. **Live tunable console.** A browser dashboard streaming world state over a
   WebSocket, with controls bound to every field of `ScenarioParams`,
   `VehicleParams`, and `GuidanceParams` — speeds, g-limits, navigation
   constant, guidance law, obstacle density, fleet size — plus live plots and
   replay-by-seed. Tune the airframe and watch Pk move.
6. **Aggregate view.** Fold the Monte Carlo output into the same dashboard:
   Pk surfaces over difficulty and `N`, and the ability to click a cell and
   replay the exact seed behind it.

### Beyond

7. **Imperfect information.** Bearing-only or noisy measurements with an EKF for
   target state estimation, and the degradation curve against perfect knowledge.
   This is the largest remaining gap between this simulator and a real seeker.
8. **Fleet assignment.** N interceptors against M targets with auction or
   Hungarian assignment and reassignment on leakers, replacing the current
   independent nearest-target choice.
9. **Performance.** Profile and port the inner loop, with a published
   steps/second figure.

## License

MIT
