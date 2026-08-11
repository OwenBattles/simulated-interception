#!/usr/bin/env python3
"""
Render a single engagement to a standalone SVG.

Deliberately stdlib-only: the figure in the README is reproducible from a
bare checkout with no plotting dependency, and the output is diffable text
rather than a binary blob.

    python scripts/plot_engagement.py --seed 0 -o results/figures/engagement.svg
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from interception import Simulation, SimulationConfig  # noqa: E402

MONO = "ui-monospace, SFMono-Regular, Menlo, Consolas, monospace"

# Styling is inlined as presentation attributes rather than a <style> block.
# GitHub sanitises SVGs embedded in markdown and strips <style>, which would
# render the README figure as unstyled black shapes.
STYLES = {
    "obstacle": 'fill="#d8d8d8" stroke="#b4b4b4" stroke-width="2"',
    "agent-path": 'fill="none" stroke="#1d4ed8" stroke-width="3"',
    "target-path": 'fill="none" stroke="#dc2626" stroke-width="3" stroke-dasharray="8 6"',
    "agent-start": 'fill="#1d4ed8" stroke="#ffffff" stroke-width="2"',
    "target-start": 'fill="#dc2626" stroke="#ffffff" stroke-width="2"',
    "intercept": 'fill="none" stroke="#111111" stroke-width="3"',
    "label": f'font-family="{MONO}" font-size="22" fill="#111111"',
    "caption": f'font-family="{MONO}" font-size="20" fill="#555555"',
}

HEADER_H = 74
FOOTER_H = 40


def record(seed, max_steps):
    """
    Run one episode, sampling every actor's position each step.

    Actor handles are re-read from the state on every iteration rather than
    captured once. They are views borrowing from the C++ State, and an
    intercept rebuilds the target list -- a handle kept across a step would
    dangle. Paths for destroyed targets simply stop growing.
    """
    sim = Simulation(SimulationConfig(seed=seed, max_steps=max_steps))
    agent_paths = [[] for _ in range(len(sim.state.agents))]
    target_paths = [[] for _ in range(len(sim.state.targets))]

    while not sim.done:
        for path, actor in zip(agent_paths, sim.state.agents):
            path.append(actor.pos.pair())
        for path, actor in zip(target_paths, sim.state.targets):
            path.append(actor.pos.pair())
        sim.step()

    return sim, agent_paths, target_paths


def polyline(points, style):
    coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return f'  <polyline {STYLES[style]} points="{coords}" />'


def circle(x, y, r, style):
    return f'  <circle {STYLES[style]} cx="{x:.1f}" cy="{y:.1f}" r="{r:.1f}" />'


def text(x, y, style, body):
    return f'  <text {STYLES[style]} x="{x:.0f}" y="{y:.0f}">{body}</text>'


def to_svg(sim, agent_paths, target_paths):
    state = sim.state
    w, h = state.width, state.height
    total_h = h + HEADER_H + FOOTER_H
    obs = sim.observation()
    miss = obs["min_miss_distance_m"]

    # Text lives in bands above and below the world so labels never sit on
    # top of the trajectories.
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {total_h:.0f}" '
        f'width="{w:.0f}" height="{total_h:.0f}" role="img">',
        f'  <rect width="{w:.0f}" height="{total_h:.0f}" fill="#ffffff" />',
        text(
            16,
            32,
            "label",
            f'seed {obs["seed"]} &#183; {obs["end_reason"]} in '
            f'{obs["elapsed_s"]:.2f} s &#183; miss {miss:.2f} m &#183; '
            f'&#916;v {obs["delta_v_mps"]:.0f} m/s',
        ),
        text(
            16,
            60,
            "caption",
            "blue = interceptor &#183; red dashed = evader &#183; "
            "grey = keep-out volumes &#183; ring = intercept",
        ),
        text(16, total_h - 14, "caption", f"{w:.0f} m &#215; {h:.0f} m engagement box"),
        f'  <g transform="translate(0 {HEADER_H})">',
        f'  <rect width="{w:.0f}" height="{h:.0f}" fill="#ffffff" '
        'stroke="#e2e2e2" stroke-width="2" />',
    ]

    for o in state.obstacles:
        parts.append(circle(o.pos.x, o.pos.y, o.radius_m, "obstacle"))

    for path in target_paths:
        parts.append(polyline(path, "target-path"))
        parts.append(circle(path[0][0], path[0][1], 9, "target-start"))
        parts.append(circle(path[-1][0], path[-1][1], 20, "intercept"))

    for path in agent_paths:
        parts.append(polyline(path, "agent-path"))
        parts.append(circle(path[0][0], path[0][1], 9, "agent-start"))

    parts += ["  </g>", "</svg>"]
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=10_000)
    parser.add_argument(
        "-o",
        "--output",
        type=pathlib.Path,
        default=pathlib.Path("results/figures/engagement.svg"),
    )
    args = parser.parse_args()

    sim, agent_paths, target_paths = record(args.seed, args.max_steps)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(to_svg(sim, agent_paths, target_paths))
    print(f"wrote {args.output} ({sim.observation()['end_reason']}, {sim.steps} steps)")


if __name__ == "__main__":
    main()
