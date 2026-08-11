"""
Helpers for the per-step telemetry the engine records.

Capture itself happens in C++ (``SimulationConfig(record_telemetry=True)``);
these are the serialisation and extraction conveniences that only the Python
side needs.
"""

import json


def frames(sim):
    """Recorded frames, or an empty list when recording was not enabled."""
    return [] if sim.telemetry is None else sim.telemetry.frames


def to_dict(sim):
    return {"summary": sim.observation(), "frames": frames(sim)}


def write_json(sim, path):
    path.write_text(json.dumps(to_dict(sim), indent=2))
    return path


def series(sim, key, agent_index=0):
    """
    Pull one scalar column out of the log, e.g. ``series(sim, "los_rate_rad_s")``.

    Frames recorded after the last target dies carry no guidance
    diagnostics, so missing keys are skipped rather than zero-filled.
    """
    out = []
    for frame in frames(sim):
        if agent_index >= len(frame["agents"]):
            continue
        row = frame["agents"][agent_index]
        if key in row:
            out.append((frame["t"], row[key]))
    return out
