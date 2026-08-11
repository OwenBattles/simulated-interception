import pygame

from .constants import (
    PIXELS_PER_METRE,
    TARGET_FPS,
    WINDOW_HEIGHT_PX,
    WINDOW_WIDTH_PX,
)
from .render import draw_world
from ._core import Simulation, SimulationConfig

HELP = "space pause | r reset | n step | p probes | esc quit"


class View:
    """
    Pygame front-end: window, input, fixed frame rate.

    Holds no simulation logic -- it steps a :class:`Simulation` and draws
    whatever state comes back.
    """

    def __init__(self, simulation=None, ppm=PIXELS_PER_METRE):
        pygame.init()
        pygame.display.set_caption("interception simulator")
        self.ppm = ppm
        self.sim = simulation or Simulation(SimulationConfig())
        self.screen = pygame.display.set_mode(
            (WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX), pygame.DOUBLEBUF
        )
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 14)
        self.running = False
        self.paused = False
        self.show_probes = True

    @property
    def state(self):
        return self.sim.state

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_ESCAPE, pygame.K_q):
                self.running = False
            elif event.key == pygame.K_SPACE:
                self.paused = not self.paused
            elif event.key == pygame.K_r:
                self.sim.reset()
            elif event.key == pygame.K_n:
                self.sim.step()  # single-step while paused
            elif event.key == pygame.K_p:
                self.show_probes = not self.show_probes

    def draw_hud(self):
        obs = self.sim.observation()
        miss = obs["min_miss_distance_m"]
        lines = [
            f"seed {obs['seed']}  t {obs['elapsed_s']:7.2f}s  step {obs['step']}",
            f"targets {obs['num_targets']}  intercepts {obs['intercepts']}"
            f"  min miss {'--' if miss is None else f'{miss:.2f} m'}",
            f"delta-v {obs['delta_v_mps']:.1f} m/s"
            + (f"  [{self.sim.end_reason}]" if self.sim.done else "")
            + ("  [paused]" if self.paused else ""),
            HELP,
        ]
        for i, line in enumerate(lines):
            self.screen.blit(self.font.render(line, True, (20, 20, 20)), (8, 8 + i * 16))

    def start(self):
        self.running = True
        while self.running:
            for event in pygame.event.get():
                self.handle_event(event)

            if not self.paused:
                self.sim.step()

            draw_world(self.screen, self.state, self.ppm, self.show_probes)
            self.draw_hud()
            pygame.display.flip()
            self.clock.tick(TARGET_FPS)

        pygame.quit()
