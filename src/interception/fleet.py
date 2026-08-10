from .agent import Agent


class Fleet:
    """A group of interceptors sharing one world and one guidance law."""

    def __init__(self, num_agents, state_ref, params=None, guidance_params=None):
        self.agents = [
            Agent(state_ref, params, guidance_params) for _ in range(num_agents)
        ]

    def __len__(self):
        return len(self.agents)

    def __iter__(self):
        return iter(self.agents)
