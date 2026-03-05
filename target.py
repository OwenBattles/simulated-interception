from actor import Actor

class Target(Actor):
    def __init__(self):
        super().__init__()
        self.max_speed = 3.5 # Example max speed, can be adjusted

    def move(self, agent):
        # TODO: implement more advanced move logic
        self.pos += self.vel


