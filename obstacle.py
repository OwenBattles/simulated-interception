from actor import Actor

class Obstacle(Actor):
    def __init__(self, position, size):
        self.position = position
        self.size = size

    