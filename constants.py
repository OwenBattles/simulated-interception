# Screen / world bounds (single source of truth for 2D extent)
SCREEN_WIDTH = 1500
SCREEN_HEIGHT = 1000

# Simulation clock (one integration tick ≈ this duration; motion is per-tick today)
SIM_DT = 1.0 / 60.0
# Headless runs stop after this many steps unless overridden (0 = no limit)
DEFAULT_HEADLESS_MAX_STEPS = 10_000

# Position bounds (existing)
MAX_XPOS = 100
MIN_XPOS = -MAX_XPOS
MAX_YPOS = 100
MIN_YPOS = -MAX_YPOS

# Physics / limits
MAX_X_VEL = 2
MAX_Y_VEL = 2
MAX_X_ACC = 1
MAX_Y_ACC = 1

# Rendering
TARGET_FPS = 60
BG_COLOR = (255, 255, 255)
AGENT_COLOR = (0, 0, 255)
TARGET_COLOR = (255, 0, 0)
AGENT_RADIUS = 6
TARGET_RADIUS = 6
MAX_OBSTACLE_RADIUS = 70
MIN_OBSTACLE_RADIUS = 30

BORDER_BUFFER = 50 