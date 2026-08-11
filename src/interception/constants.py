"""
Rendering constants for the pygame view.

Physics parameters live in the C++ engine (``core/include/interception/
constants.hpp``) and reach Python through ``interception._core``. Nothing
here affects the simulation; pixels exist only on this side of the boundary.
"""

from ._core import WORLD_HEIGHT_M, WORLD_WIDTH_M

PIXELS_PER_METRE = 0.8
WINDOW_WIDTH_PX = int(WORLD_WIDTH_M * PIXELS_PER_METRE)
WINDOW_HEIGHT_PX = int(WORLD_HEIGHT_M * PIXELS_PER_METRE)
TARGET_FPS = 60

# Vehicles are drawn as fixed-size glyphs rather than to scale: a 2 m
# airframe at 0.8 px/m would be under two pixels wide. Obstacles are drawn
# to scale because they are large enough to be legible.
AGENT_GLYPH_LENGTH_PX = 14
AGENT_GLYPH_WIDTH_PX = 7
TARGET_GLYPH_RADIUS_PX = 7

BG_COLOR = (255, 255, 255)
AGENT_COLOR = (0, 0, 255)
TARGET_COLOR = (255, 0, 0)
OBSTACLE_COLOR = (128, 128, 128)
PROBE_COLOR = (0, 200, 0)
