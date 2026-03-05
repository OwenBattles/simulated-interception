from agent import Agent

class Fleet():
    def __init__(self, num_agents):
        self.agents = [Agent() for _ in range(num_agents)]