import argparse
import json
import statistics

from .constants import DEFAULT_HEADLESS_MAX_STEPS, WORLD_HEIGHT_M, WORLD_WIDTH_M
from .simulation import EpisodeEnd, Simulation, SimulationConfig, run_headless


def build_parser():
    parser = argparse.ArgumentParser(
        prog="interception",
        description="2D counter-UAS interception simulator",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without a window until SUCCESS or TIMEOUT",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="RNG seed (omit for a random seed, which is still reported)",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=DEFAULT_HEADLESS_MAX_STEPS,
        help=f"episode step cap for headless runs (default: {DEFAULT_HEADLESS_MAX_STEPS})",
    )
    parser.add_argument("--agents", type=int, default=1, help="number of interceptors")
    parser.add_argument("--targets", type=int, default=1, help="number of targets")
    parser.add_argument(
        "--trials",
        type=int,
        default=0,
        help="run N headless episodes on seeds [seed, seed+N) and print aggregates",
    )
    parser.add_argument(
        "--json", action="store_true", help="emit machine-readable JSON output"
    )
    return parser


def run_batch(args):
    """Sweep consecutive seeds and summarise the engagement statistics."""
    base_seed = 0 if args.seed is None else args.seed
    episodes = []
    for offset in range(args.trials):
        sim = run_headless(
            seed=base_seed + offset,
            max_steps=args.max_steps,
            num_agents=args.agents,
            num_targets=args.targets,
        )
        episodes.append(sim.observation())

    successes = [e for e in episodes if e["end_reason"] == EpisodeEnd.SUCCESS.value]
    misses = [e["min_miss_distance_m"] for e in episodes if e["min_miss_distance_m"] is not None]
    summary = {
        "trials": len(episodes),
        "seeds": [base_seed, base_seed + args.trials - 1],
        "success_rate": round(len(successes) / len(episodes), 4) if episodes else 0.0,
        "mean_time_to_intercept_s": (
            round(statistics.fmean(e["elapsed_s"] for e in successes), 3)
            if successes
            else None
        ),
        "median_time_to_intercept_s": (
            round(statistics.median(e["elapsed_s"] for e in successes), 3)
            if successes
            else None
        ),
        "mean_min_miss_distance_m": (
            round(statistics.fmean(misses), 4) if misses else None
        ),
        "mean_delta_v_mps": (
            round(statistics.fmean(e["delta_v_mps"] for e in episodes), 2)
            if episodes
            else None
        ),
    }

    if args.json:
        print(json.dumps({"summary": summary, "episodes": episodes}, indent=2))
        return

    print(f"trials                  {summary['trials']} (seeds {base_seed}..{base_seed + args.trials - 1})")
    print(f"success rate            {summary['success_rate'] * 100:.1f}%")
    print(f"mean time to intercept  {_fmt(summary['mean_time_to_intercept_s'], 's')}")
    print(f"median t.t.i.           {_fmt(summary['median_time_to_intercept_s'], 's')}")
    print(f"mean min miss distance  {_fmt(summary['mean_min_miss_distance_m'], 'm')}")
    print(f"mean delta-v            {_fmt(summary['mean_delta_v_mps'], 'm/s')}")


def _fmt(value, unit):
    return "n/a" if value is None else f"{value} {unit}"


def run_single(args):
    sim = run_headless(
        seed=args.seed,
        max_steps=args.max_steps,
        num_agents=args.agents,
        num_targets=args.targets,
    )
    obs = sim.observation()
    if args.json:
        print(json.dumps(obs, indent=2))
        return
    miss = obs["min_miss_distance_m"]
    print(
        f"reason={obs['end_reason']} seed={obs['seed']} steps={obs['step']} "
        f"t={obs['elapsed_s']}s intercepts={obs['intercepts']} "
        f"min_miss={'n/a' if miss is None else f'{miss} m'} "
        f"delta_v={obs['delta_v_mps']} m/s"
    )


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.trials > 0:
        run_batch(args)
        return
    if args.headless:
        run_single(args)
        return

    # Imported lazily so the core stays usable without pygame installed.
    from .view import View

    cfg = SimulationConfig(
        world_width=WORLD_WIDTH_M,
        world_height=WORLD_HEIGHT_M,
        max_steps=0,  # interactive runs never time out
        seed=args.seed,
        num_agents=args.agents,
        num_targets=args.targets,
    )
    View(Simulation(cfg)).start()


if __name__ == "__main__":
    main()
