"""
Per-step telemetry capture.

``Simulation.observation()`` summarises an episode after it ends, which is
all a Monte Carlo sweep needs. Anything that plots an engagement -- a
guidance comparison, or the planned dashboard -- needs the time series
instead: how range, closing speed, and LOS rate evolve, and what the
guidance law was commanding at each instant.

Recording is opt-in because it allocates a dict per entity per step, which
roughly halves throughput on a large sweep.
"""

import json


class TelemetryRecorder:
    """Row-oriented frame log. One frame per simulation step."""

    def __init__(self):
        self.frames = []

    def capture(self, sim):
        state = sim.state
        self.frames.append(
            {
                "t": round(sim.elapsed_s, 4),
                "step": sim.steps,
                "agents": [self._agent_row(a) for a in state.agents],
                "targets": [self._actor_row(t) for t in state.targets],
            }
        )

    def _actor_row(self, actor):
        return {
            "x": round(actor.pos.x, 3),
            "y": round(actor.pos.y, 3),
            "vx": round(actor.vel.x, 3),
            "vy": round(actor.vel.y, 3),
            "speed_mps": round(actor.vel.magnitude(), 3),
        }

    def _agent_row(self, agent):
        row = self._actor_row(agent)
        row["delta_v_mps"] = round(agent.delta_v_mps, 3)
        row["accel_mps2"] = round(agent.acc.magnitude(), 3)
        for key, value in agent.diagnostics().items():
            row[key] = round(value, 6)
        return row

    # --- extraction -----------------------------------------------------
    def series(self, key, agent_index=0):
        """
        Pull one scalar column out of the log, e.g. ``series("los_rate_rad_s")``.

        Frames recorded after the last target dies carry no guidance
        diagnostics, so missing keys are skipped rather than zero-filled.
        """
        out = []
        for frame in self.frames:
            if agent_index >= len(frame["agents"]):
                continue
            row = frame["agents"][agent_index]
            if key in row:
                out.append((frame["t"], row[key]))
        return out

    def to_dict(self, sim):
        return {"summary": sim.observation(), "frames": self.frames}

    def write_json(self, sim, path):
        path.write_text(json.dumps(self.to_dict(sim), indent=2))
        return path
