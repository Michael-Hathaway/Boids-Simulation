from p5 import setup, draw, size, background, run, stroke, circle, fill, line
import numpy as np
import math

# Constants
MAX_SPEED = 14.0
MIN_SPEED = 5.0
HEIGHT = 1000
WIDTH = 1000
PERCEPTION_DISTANCE = 100
SEPERATION_DISTANCE = 60
MAX_COHESION = 5.0
MAX_SEPERATION = 5.0
WALL_FACTOR = 0.45
WALL_FORCE = 5.0
WALL_DISTANCE = 80


class Boid:
    def __init__(self, x_pos, y_pos):
        self.position = np.asarray([x_pos, y_pos])
        self.velocity = np.asarray((np.random.rand(2) - 0.5)*10)
        self.acceleration = np.asarray((np.random.rand(2) - 0.5)/2)

    @ staticmethod
    def get_magnitude(x, y):
        # get magnitude of x, y vector
        return math.sqrt((x**2)+(y**2))

    def show(self):
        # show the boid on the grid
        stroke(255)
        fill(255)
        circle(self.position, 7)

    def avoid_wall_collision(self):
        # change velocity when approaching a wall in order to
        # avoid a collision
        if self.position[0] >= WIDTH - WALL_DISTANCE:
            self.velocity[0] = self.velocity[0] - WALL_FORCE * WALL_FACTOR
        elif self.position[0] <= WALL_DISTANCE:
            self.velocity[0] = self.velocity[0] + WALL_FORCE * WALL_FACTOR

        if self.position[1] >= HEIGHT - WALL_DISTANCE:
            self.velocity[1] = self.velocity[1] - WALL_FORCE * WALL_FACTOR
        elif self.position[1] <= WALL_DISTANCE:
            self.velocity[1] = self.velocity[1] + WALL_FORCE * WALL_FACTOR

    def handle_edges(self):
        # when boid reached edge of screen, jump to other side
        # replace later with handling wall collisions
        if self.position[0] > WIDTH:
            self.position[0] = 0
        elif self.position[0] < 0:
            self.position[0] = WIDTH

        if self.position[1] > HEIGHT:
            self.position[1] = 0
        elif self.position[1] < 0:
            self.position[1] = HEIGHT

    def reset_acceleration(self):
        # set boid acceleration to zero
        self.acceleration = np.asarray([0.0, 0.0])

    def control_speed(self):
        # limit the boid speed based on MAX_SPEED
        if np.linalg.norm(self.velocity) > MAX_SPEED:
            self.velocity = (self.velocity /
                             np.linalg.norm(self.velocity)) * MAX_SPEED

    def align_to_neighbors(self, boids):
        # adjust movement based on neighbors
        steering = np.asarray([0.0, 0.0])
        average_velocity = np.asarray([0.0, 0.0])
        total_neighbors = 0

        for boid in boids:
            x_distance = self.position[0] - boid.position[0]
            y_distance = self.position[1] - boid.position[1]
            distance = self.get_magnitude(x_distance, y_distance)

            if distance <= PERCEPTION_DISTANCE:
                total_neighbors += 1
                average_velocity += boid.velocity

        if total_neighbors > 0:
            # only adjust if boids in PERCEPTION_DISTANCE
            average_velocity /= total_neighbors
            average_velocity = (average_velocity /
                                np.linalg.norm(average_velocity)) * MAX_SPEED
            steering = average_velocity - self.velocity

        self.acceleration += steering

    def maintain_group_cohesion(self, flock):
        steering = np.asarray([0.0, 0.0])
        average_center_of_mass = np.asarray([0.0, 0.0])
        total_neighbors = 0

        for boid in flock:
            if np.linalg.norm(boid.position - self.position) < PERCEPTION_DISTANCE:
                average_center_of_mass += boid.position
                total_neighbors += 1

        if total_neighbors > 0:
            average_center_of_mass /= total_neighbors
            direction_to_center = average_center_of_mass - self.position

            if np.linalg.norm(direction_to_center) > 0:
                direction_to_center = (direction_to_center / np.linalg.norm(direction_to_center)
                                       ) * MAX_SPEED

            steering = direction_to_center - self.velocity

            if np.linalg.norm(steering) > MAX_COHESION:
                steering = (steering / np.linalg.norm(steering)) * MAX_COHESION

        self.acceleration += steering

    def maintain_group_seperation(self, flock):
        steering = np.asarray([0.0, 0.0])
        average_vector = np.asarray([0.0, 0.0])
        total_neighbors = 0

        for boid in flock:
            x_difference = self.position[0] - boid.position[0]
            y_difference = self.position[1] - boid.position[1]
            distance = self.get_magnitude(x_difference, y_difference)

            if (x_difference != 0 or y_difference != 0) and distance <= SEPERATION_DISTANCE:
                difference = self.position - boid.position
                difference = difference / distance
                average_vector += difference
                total_neighbors += 1

        if total_neighbors > 0:
            average_vector /= total_neighbors

            if np.linalg.norm(steering) > 0:
                average_vector = (average_vector / np.linalg.norm(steering)
                                  ) * MAX_SPEED

            steering = average_vector - self.velocity

            if np.linalg.norm(steering) > MAX_SEPERATION:
                steering = (steering / np.linalg.norm(steering)) * \
                    MAX_SEPERATION

        self.acceleration += steering

    def update_position(self):
        # update the position of the boid
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]

        # update the velocity of the boid
        self.velocity[0] += self.acceleration[0]
        self.velocity[1] += self.acceleration[1]

        # limit the speed to the MAX_SPEED
        self.control_speed()

        self.reset_acceleration()
