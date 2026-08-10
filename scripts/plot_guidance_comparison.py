#!/usr/bin/env python3
"""
Plot line-of-sight rate against time for each guidance law.

This is the chart that shows *why* proportional navigation is the right
answer. A collision course is exactly the condition ``lambda_dot == 0``, so
the LOS-rate trace is a direct read-out of how well a law is doing:

- pure pursuit lets it grow without bound (tail chase),
- lead pursuit holds it down but never nulls it,
- PN drives it toward zero,
- APN nulls the residual PN leaves against an accelerating target.

The engagement is a steady 3 g turning target so all four laws differ; with
a non-manoeuvring target APN and PN are identical by construction.

Stdlib only -- see scripts/plot_engagement.py for why.

    python scripts/plot_guidance_comparison.py
"""

import argparse
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from interception.agent import Agent  # noqa: E402
from interception.guidance import engagement_geometry  # noqa: E402
from interception.params import GuidanceParams, default_interceptor  # noqa: E402
from interception.vector import Vector  # noqa: E402

MONO = "ui-monospace, SFMono-Regular, Menlo, Consolas, monospace"

# Categorical slots 1-4 of the validated default palette, in fixed order.
# Every series is also direct-labelled: aqua and yellow sit below 3:1 on this
# surface, so colour alone is not carrying identity.
SERIES = [
    ("pursuit", "#2a78d6", "pure pursuit"),
    ("lead", "#eb6834", "lead pursuit"),
    ("pn", "#1baf7a", "proportional nav"),
    ("apn", "#eda100", "augmented PN"),
]

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_MUTED = "#52514e"
GRID = "#e4e3df"

W, H = 1180, 620
PAD_L, PAD_R, PAD_T, PAD_B = 78, 258, 112, 68
PLOT_W = W - PAD_L - PAD_R
PLOT_H = H - PAD_T - PAD_B

# PN and APN converge to nearly the same LOS rate, so their end-of-line
# labels would sit on top of each other. Labels go in a right-hand gutter
# and are pushed apart to this spacing, with leader lines back to the mark.
LABEL_GUTTER_X = PAD_L + PLOT_W + 18
LABEL_MIN_GAP = 44


class TurningTarget:
    """Constant-speed target in a steady turn (constant lateral g)."""

    def __init__(self, pos, vel, lateral_accel_mps2):
        self.pos = pos
        self.vel = vel
        self.lateral = lateral_accel_mps2
        self.acc = Vector()
        self.hit_radius_m = 1.0

    def step(self, dt):
        normal = self.vel.normalize().perpendicular()
        self.acc = normal * self.lateral
        self.vel = self.vel + self.acc * dt
        self.pos = self.pos + self.vel * dt


class FakeWorld:
    def __init__(self):
        self.rng = random.Random(0)
        self.width = self.height = 1e7
        self.obstacles = []
        self.targets = []


def fly(law, nav_constant, dt, max_steps, plot_gate_m):
    """
    Fly one engagement and return its LOS-rate trace.

    ``lambda_dot`` carries ``1/R^2``, so it diverges in the last fraction of
    a second regardless of how well the law is doing -- lead pursuit exits
    this engagement at 38 rad/s. That terminal spike is a real singularity,
    not a defect, but plotting it compresses every meaningful curve into the
    baseline. The trace is therefore cut at ``plot_gate_m`` while the flight
    itself continues to intercept, so the reported times and delta-v are the
    true end-to-end figures.
    """
    world = FakeWorld()
    agent = Agent(
        world, default_interceptor(), GuidanceParams(law=law, nav_constant=nav_constant)
    )
    agent.pos = Vector(5_000, 5_000)
    agent.vel = Vector(120, 0)
    agent.reorient()

    target = TurningTarget(Vector(6_200, 5_000), Vector(0, 80), lateral_accel_mps2=30.0)
    world.targets = [target]

    times, rates = [], []
    intercepted = False
    elapsed = 0.0
    for step in range(max_steps):
        _, range_m, _, rate = engagement_geometry(agent, target)
        elapsed = step * dt
        if range_m >= plot_gate_m:
            times.append(elapsed)
            rates.append(abs(rate))
        if range_m < agent.hit_radius_m + target.hit_radius_m:
            intercepted = True
            break
        agent.step(dt)
        target.step(dt)

    return times, rates, intercepted, agent.delta_v_mps, elapsed


def nice_ceiling(value):
    """Round up to a clean axis maximum."""
    if value <= 0:
        return 1.0
    magnitude = 10 ** math.floor(math.log10(value))
    for step in (1, 1.5, 2, 2.5, 5, 10):
        if value <= step * magnitude:
            return step * magnitude
    return 10 * magnitude


def svg_text(x, y, body, size=15, fill=INK, anchor="start", weight="normal"):
    return (
        f'  <text x="{x:.1f}" y="{y:.1f}" font-family="{MONO}" font-size="{size}" '
        f'fill="{fill}" text-anchor="{anchor}" font-weight="{weight}">{body}</text>'
    )


def build(runs, dt):
    x_max = nice_ceiling(max(r["times"][-1] for r in runs))
    y_max = nice_ceiling(max(max(r["rates"]) for r in runs))

    def sx(t):
        return PAD_L + (t / x_max) * PLOT_W

    def sy(v):
        return PAD_T + PLOT_H - (v / y_max) * PLOT_H

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
        f'width="{W}" height="{H}" role="img" '
        f'aria-label="Line-of-sight rate against time for four guidance laws">',
        f'  <rect width="{W}" height="{H}" fill="{SURFACE}" />',
        svg_text(PAD_L, 40, "Line-of-sight rate by guidance law", size=21, weight="600"),
        svg_text(
            PAD_L,
            64,
            "Interceptor vs a 3 g turning target. A collision course is "
            "|&#955;&#775;| = 0, so lower is better.",
            size=15,
            fill=INK_MUTED,
        ),
        svg_text(
            PAD_L,
            84,
            "Trace cut at 50 m range, where |&#955;&#775;| diverges as 1/R&#178;; "
            "flights continue to intercept.",
            size=15,
            fill=INK_MUTED,
        ),
    ]

    # Recessive horizontal grid with value labels.
    for i in range(6):
        value = y_max * i / 5
        y = sy(value)
        parts.append(
            f'  <line x1="{PAD_L}" y1="{y:.1f}" x2="{PAD_L + PLOT_W}" y2="{y:.1f}" '
            f'stroke="{GRID}" stroke-width="1" />'
        )
        parts.append(
            svg_text(PAD_L - 12, y + 5, f"{value:.02f}", size=13, fill=INK_MUTED, anchor="end")
        )

    # Five intervals keeps the labels on round numbers; six would put ticks
    # at 1.67, 3.33, ... and round them to an uneven-looking 0 2 3 5 7 8 10.
    for i in range(6):
        t = x_max * i / 5
        parts.append(
            svg_text(
                sx(t),
                PAD_T + PLOT_H + 26,
                f"{t:g}",
                size=13,
                fill=INK_MUTED,
                anchor="middle",
            )
        )

    parts.append(
        svg_text(
            PAD_L + PLOT_W / 2,
            PAD_T + PLOT_H + 52,
            "time (s)",
            size=14,
            fill=INK_MUTED,
            anchor="middle",
        )
    )
    parts.append(
        f'  <text x="22" y="{PAD_T + PLOT_H / 2:.1f}" font-family="{MONO}" '
        f'font-size="14" fill="{INK_MUTED}" text-anchor="middle" '
        f'transform="rotate(-90 22 {PAD_T + PLOT_H / 2:.1f})">'
        "|LOS rate| (rad/s)</text>"
    )

    # Series: 2px lines, each direct-labelled in the gutter.
    marks = []
    for run in runs:
        pts = " ".join(
            f"{sx(t):.1f},{sy(v):.1f}" for t, v in zip(run["times"], run["rates"])
        )
        parts.append(
            f'  <polyline fill="none" stroke="{run["color"]}" stroke-width="2" '
            f'stroke-linejoin="round" points="{pts}" />'
        )
        end_x, end_y = sx(run["times"][-1]), sy(run["rates"][-1])
        parts.append(
            f'  <circle cx="{end_x:.1f}" cy="{end_y:.1f}" r="4.5" '
            f'fill="{run["color"]}" stroke="{SURFACE}" stroke-width="2" />'
        )
        marks.append({"run": run, "x": end_x, "y": end_y, "label_y": end_y})

    # Push overlapping labels apart, top to bottom, then clamp to the canvas.
    marks.sort(key=lambda m: m["label_y"])
    for previous, current in zip(marks, marks[1:]):
        gap = current["label_y"] - previous["label_y"]
        if gap < LABEL_MIN_GAP:
            current["label_y"] = previous["label_y"] + LABEL_MIN_GAP
    overflow = marks[-1]["label_y"] - (PAD_T + PLOT_H)
    if overflow > 0:
        for mark in marks:
            mark["label_y"] -= overflow

    for mark in marks:
        run, label_y = mark["run"], mark["label_y"]
        parts.append(
            f'  <path d="M {mark["x"]:.1f} {mark["y"]:.1f} '
            f'L {LABEL_GUTTER_X - 8:.1f} {label_y:.1f}" fill="none" '
            f'stroke="{run["color"]}" stroke-width="1" opacity="0.45" />'
        )
        note = "intercept" if run["intercepted"] else "no intercept"
        parts.append(
            svg_text(LABEL_GUTTER_X, label_y + 1, run["label"], size=14, fill=INK)
        )
        parts.append(
            svg_text(
                LABEL_GUTTER_X,
                label_y + 19,
                f"{note} {run['elapsed']:.2f}s &#183; &#916;v {run['delta_v']:.0f} m/s",
                size=12,
                fill=INK_MUTED,
            )
        )

    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-constant", type=float, default=4.0)
    parser.add_argument("--dt", type=float, default=1 / 60)
    parser.add_argument("--max-steps", type=int, default=4000)
    parser.add_argument(
        "--plot-gate",
        type=float,
        default=50.0,
        help="stop plotting inside this range, where lambda_dot diverges (m)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=pathlib.Path,
        default=pathlib.Path("results/figures/guidance-comparison.svg"),
    )
    args = parser.parse_args()

    runs = []
    for law, color, label in SERIES:
        times, rates, intercepted, delta_v, elapsed = fly(
            law, args.nav_constant, args.dt, args.max_steps, args.plot_gate
        )
        runs.append(
            {
                "law": law,
                "color": color,
                "label": label,
                "times": times,
                "rates": rates,
                "intercepted": intercepted,
                "delta_v": delta_v,
                "elapsed": elapsed,
            }
        )
        end = "intercept" if intercepted else "NO INTERCEPT"
        print(
            f"{law:8s} {end:12s} t={elapsed:6.2f}s  "
            f"|lambda_dot| at {args.plot_gate:.0f} m gate={rates[-1]:.5f}  dv={delta_v:7.1f}"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build(runs, args.dt))
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
