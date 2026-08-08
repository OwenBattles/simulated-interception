"""
Single source of truth for world, vehicle, and rendering parameters.

UNITS
-----
The simulation core is SI throughout: metres, seconds, kilograms, newtons,
radians. Pixels exist only in the rendering layer (``render``/``view``),
which converts with ``PIXELS_PER_METRE``. No physics quantity below is
expressed per-tick -- every rate is per-second and is multiplied by ``dt``
at integration time, so behaviour is independent of the frame rate.

SCENARIO
--------
Numbers describe a short-range counter-UAS engagement: a fixed-wing
interceptor running down a manoeuvring quadrotor inside a 1.5 km x 1.0 km
box containing no-fly volumes (modelled as circular obstacles).
"""

# --- World --------------------------------------------------------------
WORLD_WIDTH_M = 1500.0
WORLD_HEIGHT_M = 1000.0

# --- Simulation clock ---------------------------------------------------
SIM_DT = 1.0 / 60.0  # s, fixed physics timestep
DEFAULT_HEADLESS_MAX_STEPS = 10_000  # 10000 * dt ~= 167 s of flight

# --- Interceptor --------------------------------------------------------
AGENT_MASS_KG = 5.0
AGENT_MAX_SPEED_MPS = 120.0
AGENT_MAX_FORCE_N = 1000.0  # 1000 N / 5 kg = 200 m/s^2 ~= 20 g
AGENT_HIT_RADIUS_M = 2.0  # capture radius
AGENT_PROBE_LOOKAHEAD_S = 0.8  # probe sits this many seconds ahead
# Turn radius at full speed: v^2/a = 120^2/200 = 72 m

# --- Target -------------------------------------------------------------
TARGET_MASS_KG = 3.0
TARGET_MAX_SPEED_MPS = 80.0
TARGET_MAX_FORCE_N = 300.0  # 300 N / 3 kg = 100 m/s^2 ~= 10 g
TARGET_HIT_RADIUS_M = 1.0
TARGET_PROBE_LOOKAHEAD_S = 0.8
# Wander angle is a Wiener process, so its scale is rad per sqrt(second).
TARGET_WANDER_SIGMA_RAD_PER_SQRT_S = 0.6
TARGET_WANDER_MAX_RAD = 1.2  # clamp so the evader cannot invert its heading
TARGET_WANDER_CIRCLE_DIST_M = 60.0
TARGET_WANDER_CIRCLE_RADIUS_M = 60.0

# --- Sensing ------------------------------------------------------------
PROBE_RADIUS_M = 25.0
AVOIDANCE_BRAKING_WEIGHT = 0.2  # fraction of max force spent decelerating

# --- Obstacles ----------------------------------------------------------
MIN_OBSTACLE_RADIUS_M = 30.0
MAX_OBSTACLE_RADIUS_M = 70.0
MIN_OBSTACLE_COUNT = 5
MAX_OBSTACLE_COUNT = 10

# --- Rendering (pixels live here and nowhere else) ----------------------
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
