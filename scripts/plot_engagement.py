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

from interception.simulation import Simulation, SimulationConfig  # noqa: E402

STYLE = """
  .obstacle { fill: #d8d8d8; stroke: #b4b4b4; stroke-width: 2; }
  .agent-path { fill: none; stroke: #1d4ed8; stroke-width: 3; }
  .target-path { fill: none; stroke: #dc2626; stroke-width: 3; stroke-dasharray: 8 6; }
  .start { stroke-width: 2; }
  .intercept { fill: none; stroke: #111; stroke-width: 3; }
  .label { font: 22px ui-monospace, monospace; fill: #111; }
  .caption { font: 20px ui-monospace, monospace; fill: #555; }
"""


def record(seed, max_steps):
    """Run one episode, sampling every actor's position each step."""
    sim = Simulation(SimulationConfig(seed=seed, max_steps=max_steps))
    agent_paths = [[] for _ in sim.state.agents]
    target_paths = [[] for _ in sim.state.targets]
    agents = list(sim.state.agents)
    targets = list(sim.state.targets)

    while not sim.done:
        for path, actor in zip(agent_paths, agents):
            path.append(actor.pos.pair())
        for path, actor in zip(target_paths, targets):
            path.append(actor.pos.pair())
        sim.step()

    return sim, agent_paths, target_paths


def polyline(points, css_class):
    coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return f'  <polyline class="{css_class}" points="{coords}" />'


HEADER_H = 74
FOOTER_H = 40


def to_svg(sim, agent_paths, target_paths):
    state = sim.state
    w, h = state.width, state.height
    total_h = h + HEADER_H + FOOTER_H
    obs = sim.observation()
    miss = obs["min_miss_distance_m"]

    # Text lives in its own bands above and below the world so labels never
    # sit on top of the trajectories.
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {total_h:.0f}" '
        f'width="{w:.0f}" height="{total_h:.0f}" role="img">',
        f"  <style>{STYLE}</style>",
        f'  <rect width="{w:.0f}" height="{total_h:.0f}" fill="#fff" />',
        f'  <text class="label" x="16" y="32">seed {obs["seed"]} &#183; '
        f'{obs["end_reason"]} in {obs["elapsed_s"]:.2f} s &#183; '
        f'miss {miss:.2f} m &#183; &#916;v {obs["delta_v_mps"]:.0f} m/s</text>',
        '  <text class="caption" x="16" y="60">'
        "blue = interceptor &#183; red dashed = evader &#183; "
        "grey = keep-out volumes &#183; ring = intercept</text>",
        f'  <text class="caption" x="16" y="{total_h - 14:.0f}">'
        f"{w:.0f} m &#215; {h:.0f} m engagement box</text>",
        f'  <g transform="translate(0 {HEADER_H})">',
        f'  <rect width="{w:.0f}" height="{h:.0f}" fill="#fff" '
        'stroke="#e2e2e2" stroke-width="2" />',
    ]

    for o in state.obstacles:
        parts.append(
            f'  <circle class="obstacle" cx="{o.pos.x:.1f}" cy="{o.pos.y:.1f}" '
            f'r="{o.radius_m:.1f}" />'
        )

    for path in target_paths:
        parts.append(polyline(path, "target-path"))
        x, y = path[0]
        parts.append(f'  <circle class="start" cx="{x:.1f}" cy="{y:.1f}" r="9" fill="#dc2626" />')
        ix, iy = path[-1]
        parts.append(f'  <circle class="intercept" cx="{ix:.1f}" cy="{iy:.1f}" r="20" />')

    for path in agent_paths:
        parts.append(polyline(path, "agent-path"))
        x, y = path[0]
        parts.append(f'  <circle class="start" cx="{x:.1f}" cy="{y:.1f}" r="9" fill="#1d4ed8" />')

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
