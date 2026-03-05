from actor import Actor

class Target(Actor):
    def __init__(self):
        super().__init__()

    def move(self):
        # TODO: implement more advanced move logic
        self.pos += self.vel