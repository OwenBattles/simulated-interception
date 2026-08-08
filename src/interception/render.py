"""
All pygame drawing lives here.

Keeping it out of the entity classes is what lets the model run headless in
CI without SDL: nothing under ``interception`` imports pygame except this
module and :mod:`interception.view`.

World coordinates are metres with +y down; screen coordinates are pixels
with +y down, so the transform is a uniform scale with no flip.
"""

import pygame

from .constants import (
    AGENT_COLOR,
    AGENT_GLYPH_LENGTH_PX,
    AGENT_GLYPH_WIDTH_PX,
    BG_COLOR,
    OBSTACLE_COLOR,
    PIXELS_PER_METRE,
    PROBE_COLOR,
    TARGET_COLOR,
    TARGET_GLYPH_RADIUS_PX,
)


def to_screen(vec, ppm):
    return (int(vec.x * ppm), int(vec.y * ppm))


def draw_obstacle(screen, obstacle, ppm):
    # Obstacles are drawn to scale: they are large enough to read.
    pygame.draw.circle(
        screen,
        OBSTACLE_COLOR,
        to_screen(obstacle.pos, ppm),
        max(1, int(obstacle.radius_m * ppm)),
    )


def draw_probe(screen, probe, ppm):
    pygame.draw.circle(
        screen,
        PROBE_COLOR,
        to_screen(probe.pos, ppm),
        max(1, int(probe.radius_m * ppm)),
        width=1,
    )


def draw_agent(screen, agent, ppm):
    # Fixed-size glyph, not to scale: a 2 m airframe at 0.8 px/m would be
    # under two pixels across. Tactical displays use icons for the same reason.
    cx, cy = to_screen(agent.pos, ppm)
    fwd, side = agent.forward_vec, agent.side_vec
    length, width = AGENT_GLYPH_LENGTH_PX, AGENT_GLYPH_WIDTH_PX
    points = [
        (cx + fwd.x * length, cy + fwd.y * length),
        (cx - fwd.x * length + side.x * width, cy - fwd.y * length + side.y * width),
        (cx - fwd.x * length - side.x * width, cy - fwd.y * length - side.y * width),
    ]
    pygame.draw.polygon(screen, AGENT_COLOR, points)


def draw_target(screen, target, ppm):
    pygame.draw.circle(
        screen, TARGET_COLOR, to_screen(target.pos, ppm), TARGET_GLYPH_RADIUS_PX
    )


def draw_world(screen, state, ppm=PIXELS_PER_METRE, show_probes=True):
    screen.fill(BG_COLOR)
    for obstacle in state.obstacles:
        draw_obstacle(screen, obstacle, ppm)
    for target in state.targets:
        if show_probes and target.probe is not None:
            draw_probe(screen, target.probe, ppm)
        draw_target(screen, target, ppm)
    for agent in state.agents:
        if show_probes and agent.probe is not None:
            draw_probe(screen, agent.probe, ppm)
        draw_agent(screen, agent, ppm)
