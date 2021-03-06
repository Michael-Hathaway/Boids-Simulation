from p5 import setup, draw, size, background, run, stroke, circle
import numpy as np
from boid import Boid

# Constants
WIDTH = 1500
HEIGHT = 900


def create_flock(size=30):
    # create flock of boids
    return [
        Boid(x_pos=np.random.randint(0, WIDTH),
             y_pos=np.random.randint(0, HEIGHT)) for _ in range(0, size)
    ]


def setup():
    # Setup the grid - only done once
    size(WIDTH, HEIGHT)


flock = create_flock()


def draw():
    # update the grid - done every time
    background(30, 30, 47)

    for boid in flock:
        boid.show()
        boid.align_to_neighbors(flock)
        boid.maintain_group_cohesion(flock)
        boid.maintain_group_seperation(flock)
        boid.update_position()


if __name__ == "__main__":
    run(frame_rate=60)
