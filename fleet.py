from agent import Agent

class Fleet():
    def __init__(self, num_agents, state_ref):
        self.agents = [Agent(state_ref) for _ in range(num_agents)]