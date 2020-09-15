from p5 import setup, draw, size, background, run, stroke, circle
import numpy as np
import math

# Constants
MAX_SPEED = 15.0
MIN_SPEED = 5.0
HEIGHT = 1000
WIDTH = 1000


def get_magnitude(x, y):
    # get magnitude of x, y vector
    return math.sqrt((x**2)+(y**2))


class Boid:
    def __init__(self, x_pos, y_pos):
        self.position = [x_pos, y_pos]
        self.velocity = [
            np.random.randint(MIN_SPEED, MAX_SPEED) *
            np.random.choice([-1, 1]),
            np.random.randint(MIN_SPEED, MAX_SPEED) *
            np.random.choice([-1, 1]),
        ]
        self.acceleration = [0.0, 0.0]

    def show(self):
        # show the boid on the grid
        stroke(255)
        circle(self.position, 8)

    def handle_wall_collision(self):
        pass

    def update_position(self):
        # update the position and velocity of the boid
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]

        self.handle_wall_collision()
