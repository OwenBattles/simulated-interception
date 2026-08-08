from .agent import Agent


class Fleet:
    """A group of interceptors sharing one world."""

    def __init__(self, num_agents, state_ref):
        self.agents = [Agent(state_ref) for _ in range(num_agents)]

    def __len__(self):
        return len(self.agents)

    def __iter__(self):
        return iter(self.agents)
