import argparse

from constants import DEFAULT_HEADLESS_MAX_STEPS, SCREEN_HEIGHT, SCREEN_WIDTH
from simulation import Simulation, SimulationConfig, run_headless
from view import View


def main():
    parser = argparse.ArgumentParser(description="2D interception simulator")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without pygame until SUCCESS or TIMEOUT",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="RNG seed for reproducible world (headless defaults to 0 if omitted)",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help=f"Episode step cap for headless (default: {DEFAULT_HEADLESS_MAX_STEPS})",
    )
    args = parser.parse_args()

    if args.headless:
        max_steps = (
            args.max_steps
            if args.max_steps is not None
            else DEFAULT_HEADLESS_MAX_STEPS
        )
        seed = 0 if args.seed is None else args.seed
        sim = run_headless(seed=seed, max_steps=max_steps)
        print(
            "headless_done",
            f"reason={sim.end_reason.value}",
            f"steps={sim.steps}",
            f"seed={seed}",
        )
        return

    cfg = SimulationConfig(
        world_width=SCREEN_WIDTH,
        world_height=SCREEN_HEIGHT,
        max_steps=0,
        seed=args.seed,
    )
    view = View(SCREEN_WIDTH, SCREEN_HEIGHT, simulation=Simulation(cfg))
    view.start()


if __name__ == "__main__":
    main()
