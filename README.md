# Interception simulator

A 2D counter-UAS engagement simulator: an interceptor runs down a manoeuvring
target through a field of keep-out volumes, and the run is scored on the
metrics an engagement is actually judged by — probability of kill, time to
intercept, miss distance, and control effort.

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

**Guidance.** Lead pursuit — aim at where the target will be after a
first-order time-to-go, `t_go = range / interceptor max speed`, and fly there
at full speed. Exact for a head-on non-manoeuvring target, degrades in a
crossing geometry.

**Avoidance.** A disc probe is swept ahead of the nose at a distance of
`0.8 s × current speed`, so the vehicle holds a constant *reaction time*
rather than a constant reaction distance. When the probe overlaps an obstacle,
the interceptor turns away from whichever side the obstacle sits on and bleeds
a little speed to tighten the turn. Avoidance pre-empts pursuit.

## Results

500 seeds per configuration, `interception --trials 500`:

| Configuration | Success | Mean TTI | Median TTI | Mean miss | Mean Δv |
| --- | --- | --- | --- | --- | --- |
| 1 interceptor, 1 target | 100% | 8.23 s | 7.42 s | 2.30 m | 994 m/s |
| 1 interceptor, 3 targets | 100% | 21.02 s | 19.40 s | 1.90 m | 2621 m/s |
| 3 interceptors, 3 targets | 100% | 10.49 s | 9.73 s | 1.86 m | 3814 m/s |

Δv is summed across the fleet, which is why it rises with interceptor count
even as time-to-intercept falls.

**Read the 100% honestly.** The current evader flies a random-walk wander and
never reacts to being chased, so a 1.5× speed advantage is decisive and
success rate carries no information. The number that matters right now is
time-to-intercept; probability of kill only becomes a real metric once the
target evades. That is the next change — see [Roadmap](#roadmap).

## Quickstart

```bash
pip install -e .              # core only, no dependencies
interception --headless --seed 0
interception --trials 500     # seed sweep with aggregate statistics
interception --trials 100 --json > runs.json

pip install -e '.[viz]'       # adds pygame
interception                  # opens a window
```

Interactive keys: `space` pause · `n` single-step · `r` reset · `p` toggle
probes · `esc` quit.

Regenerate the figure above:

```bash
python scripts/plot_engagement.py --seed 0
```

## Design notes

**SI units, everywhere.** Every quantity in the core is metres, seconds,
kilograms, and newtons. Nothing is expressed per-tick: rates are per-second and
multiplied by `dt` at integration time, so trajectories are identical at 60 Hz
and 240 Hz. Pixels exist only in `render.py`. This is enforced by tests, not
convention — `tests/test_integration.py` runs the same manoeuvre at three
timesteps and asserts the results agree.

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
yields miss distance for free, which is the metric the whole problem is judged
on.

**Static geometry is not an actor.** Obstacles have no mass or velocity and
never step. Probes are plain sensors, not vehicles.

Modelling simplifications are catalogued in [docs/assumptions.md](docs/assumptions.md).

## Layout

```
src/interception/
  vector.py       2D vector maths
  constants.py    all world/vehicle/render parameters, SI
  actor.py        integration, orientation, bounds, obstacle avoidance
  agent.py        interceptor guidance
  target.py       evader (Reynolds wander)
  obstacle.py     static keep-out volumes
  sensor.py       forward-looking collision probe
  collision.py    swept-sphere closest approach
  state.py        world, entity lists, intercept resolution
  simulation.py   episode lifecycle, config, metrics
  fleet.py        interceptor group
  render.py       all pygame drawing
  view.py         window, input, frame loop
  cli.py          entrypoint
scripts/          reproducible figure generation
tests/            45 tests, no pygame required
docs/             modelling assumptions
```

## Testing

```bash
pip install -e '.[dev]'
pytest
```

45 tests covering vector maths, timestep independence, force and speed
clamping, swept-collision geometry, obstacle-avoidance behaviour, world
determinism, and episode lifecycle. CI runs the suite on Python 3.10–3.12 plus
a headless smoke run.

Three of the tests are regressions for bugs found in earlier revisions and are
written to fail if the fix is reverted:

- Comparing distances to obstacles passed an `Obstacle` where a `Vector` was
  expected. A short-circuiting `or` hid it until a *second* obstacle entered
  probe range, so it only surfaced in cluttered geometry.
- Intercepts mutated the actor list while the step loop was iterating it,
  silently skipping actors on any tick where a target died.
- `reset()` reseeded from OS entropy when called without an argument, quietly
  breaking the reproducibility guarantee.

## Roadmap

Ordered by how much each changes what the simulator can tell you.

1. **Proportional navigation.** Replace lead pursuit with true PN
   (`a_cmd = N · λ̇ · V_c`) and augmented PN, keeping pursuit as a baseline, and
   benchmark the three against each other.
2. **Adversarial evader.** Evasive manoeuvres — weave, jink on detection — with
   a difficulty parameter, so probability of kill becomes a curve rather than a
   constant.
3. **Monte Carlo harness.** Sweep difficulty and guidance law across thousands
   of seeded runs; publish Pk curves, miss-distance CDFs, and Δv budgets.
4. **Imperfect information.** Bearing-only or noisy measurements with an EKF
   for target state estimation, and the degradation curve against perfect
   knowledge.
5. **Fleet assignment.** N interceptors against M targets with auction or
   Hungarian assignment and reassignment on leakers, replacing the current
   independent nearest-target choice.
6. **Performance.** Profile and port the inner loop, with a published
   steps/second figure.

## License

MIT
