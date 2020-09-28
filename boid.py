from p5 import stroke, circle, fill, line
import numpy as np
import math

# Constants
MAX_SPEED = 12.0
MIN_SPEED = 6.0

HEIGHT = 900
WIDTH = 1500

PERCEPTION_DISTANCE = 100.0
SEPERATION_DISTANCE = 40.0

MAX_COHESION = 5.0
MAX_SEPERATION = 5.0

WALL_FACTOR = 0.45
WALL_FORCE = 5.0
WALL_DISTANCE = 40.0

ALIGNMENT_FACTOR = 0.25
COHESION_FACTOR = 0.05
SEPERATION_FACTOR = 0.6


class Boid:
    def __init__(self, x_pos, y_pos):
        self.position = np.asarray([x_pos, y_pos])
        self.velocity = np.asarray((np.random.rand(2) - 0.5)*10)
        self.acceleration = np.asarray((np.random.rand(2) - 0.5)/2)

    @ staticmethod
    def get_magnitude(x, y):
        # get magnitude of x, y vector
        return math.sqrt((x**2)+(y**2))

    def control_velocity(self, max_mag, min_mag=0.0):
        mag = self.get_magnitude(self.velocity[0], self.velocity[1])
        if mag == 0:
            return
        elif mag > max_mag:
            normalizing_factor = max_mag / mag
        elif mag < min_mag:
            normalizing_factor = min_mag / mag
        else:
            return

        self.velocity = [value * normalizing_factor for value in self.velocity]

    def show(self):
        # find line showing boid direction
        y_angle = math.atan(self.velocity[1] / self.velocity[0])
        x_angle = math.atan(self.velocity[0] / self.velocity[1])
        larger = x_angle if x_angle > y_angle else y_angle
        lsign = -1 if larger < 0 else 1

        norm_x = x_angle / (10.0 * lsign)
        norm_y = y_angle / (10.0 * lsign)

        x_mult = -1 if self.velocity[0] < 0 else 1
        y_mult = -1 if self.velocity[1] < 0 else 1

        x_point_coord = self.position[0] + (x_mult * norm_x * 100)
        y_point_coord = self.position[1] + (y_mult * norm_y * 100)

        # show the boid on the grid
        stroke(255)
        fill(255)
        circle(self.position, 9)
        stroke("red")
        line(self.position, (x_point_coord, y_point_coord))

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

    def align_to_neighbors(self, flock):
        # adjust movement based on neighbors
        steering = np.asarray([0.0, 0.0])
        average_velocity = np.asarray([0.0, 0.0])
        total_neighbors = 0

        for boid in flock:
            x_distance = self.position[0] - boid.position[0]
            y_distance = self.position[1] - boid.position[1]
            distance = self.get_magnitude(x_distance, y_distance)

            if distance <= PERCEPTION_DISTANCE:
                total_neighbors += 1
                average_velocity += boid.velocity

        if total_neighbors > 0:
            # only adjust if flock in PERCEPTION_DISTANCE
            average_velocity /= total_neighbors
            average_velocity = (average_velocity /
                                np.linalg.norm(average_velocity)) * MAX_SPEED
            steering = average_velocity - self.velocity

        self.acceleration += (steering * ALIGNMENT_FACTOR)

    def maintain_group_cohesion(self, flock):
        steering = np.asarray([0.0, 0.0])
        average_center_of_mass = np.asarray([0.0, 0.0])
        total_neighbors = 0

        for boid in flock:
            x_distance = self.position[0] - boid.position[0]
            y_distance = self.position[1] - boid.position[1]
            distance = self.get_magnitude(x_distance, y_distance)
            if distance <= PERCEPTION_DISTANCE:
                average_center_of_mass += boid.position
                total_neighbors += 1

        if total_neighbors > 0:
            average_center_of_mass /= total_neighbors
            direction_to_center = average_center_of_mass - self.position

            if np.linalg.norm(direction_to_center) > 0:
                direction_to_center = (direction_to_center / np.linalg.norm(direction_to_center)
                                       ) * MAX_SPEED

            direction_to_center = direction_to_center - self.velocity

            if np.linalg.norm(steering) > MAX_COHESION:
                direction_to_center = (
                    direction_to_center / np.linalg.norm(direction_to_center)) * MAX_COHESION

        self.acceleration += (direction_to_center * COHESION_FACTOR)

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
                average_vector = (average_vector / np.linalg.norm(average_vector)
                                  ) * MAX_SPEED

            average_vector = average_vector - self.velocity

            if np.linalg.norm(average_vector) > MAX_SEPERATION:
                average_vector = (average_vector / np.linalg.norm(average_vector)) * \
                    MAX_SEPERATION

        self.acceleration += (average_vector * SEPERATION_FACTOR)

    def update_position(self):
        # update the position of the boid
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]

        # update the velocity of the boid
        self.velocity[0] += self.acceleration[0]
        self.velocity[1] += self.acceleration[1]

        self.control_velocity(MAX_SPEED, MIN_SPEED)
        self.avoid_wall_collision()
        self.reset_acceleration()
