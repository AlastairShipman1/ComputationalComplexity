import numpy as np
import pygame
import config
from model.ModelUtils import is_left

""" 
File for agents
Units in metric
angular units in radians, varying 0->2pi
"""


class Actor:
    def __init__(self):
        self.speed = 5


class Vehicle:

    def __init__(self, image_path=None, initial_position=(0, 150), starting_direction_degrees=0, world=None):

        # simulation variables
        self.world = world
        self.v_long = 0
        self.v_lat = 0
        self.acc_long = 0
        self.direction = np.deg2rad(starting_direction_degrees)
        self.vel_inc = 1  # ms^-2
        self.max_acc_rate = 5  # ms^-2
        self.friction_rate = 0
        self.turn_inc = np.deg2rad(.3)  # rads per frame
        self.turn_circle = 0  # rads per frame
        self.max_turn_circle = np.deg2rad(3.5)  # rads per frame
        self.max_v = 30  # 30 is approximately 70mph
        self.dt = np.inf

        # drawing/visualisation variables
        self.image_path = image_path
        if image_path is None:
            self.image_path = config.INPUT_IMAGE_FP + '/blue_car_top_down.png'

        self.pixel_to_metres_ratio = 1
        self.actual_vehicle_size = [6, 3]
        self.draw_size = [self.actual_vehicle_size[0] * self.pixel_to_metres_ratio,
                          self.actual_vehicle_size[1] * self.pixel_to_metres_ratio]

        self.original_image = pygame.image.load(self.image_path)
        self.original_image = pygame.transform.rotate(self.original_image, 180)
        self.image = pygame.transform.smoothscale(self.original_image, self.draw_size)
        self.rotate()

        self.image_offset = self.image.get_rect().center
        self.draw_offset = 0, 0
        self.draw_scale = 1
        self.world_x = -self.image_offset[0] + initial_position[0]
        self.world_y = -self.image_offset[1] + initial_position[1]
        self.draw_x = self.world_x
        self.draw_y = self.world_y

    def get_velocity(self):
        return [self.v_long*np.cos(self.direction), self.v_long*np.sin(self.direction)]

    def get_position(self):
        return [self.world_x, self.world_y]


    # region utils
    def move(self):
        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # dt is in milliseconds.

        x_addition = self.v_long * self.dt / 1000 * np.cos(self.direction)
        self.world_x += x_addition
        y_addition = self.v_long * self.dt / 1000 * np.sin(self.direction)
        self.world_y -= y_addition

    def update(self, dt):
        self.dt = dt
        self.move()
        self.rotate(self.turn_circle)
        self.friction()

    def friction(self):
        # TODO: this should be an actual physical air resistance model.
        friction_scaler = 1
        if abs(self.v_long) < 15:
            friction_scaler = 1.5
        self.v_long -= np.sign(self.v_long) * self.friction_rate * friction_scaler
        if abs(self.v_long) < 1:
            self.v_long = 0

        # TODO: this should be dependent on the friction, not magic numbers
        turn_circle = self.turn_circle - np.sign(self.turn_circle) * self.turn_inc * 0.9
        if np.sign(self.turn_circle) != np.sign(turn_circle):
            turn_circle = 0
        self.turn_circle = turn_circle

    def draw(self, surface):
        ##### draw agent on surface#########
        self.update_offset(self.draw_offset)
        pos = (self.draw_x - self.image_offset[0], self.draw_y - self.image_offset[1])
        surface.blit(self.image, pos)

    def send_message(self, string):
        ...

    def accelerate(self, amount):
        if abs(amount) > self.vel_inc:
            amount = np.sign(amount) * self.vel_inc
        v_long = self.v_long + amount
        if abs(v_long) > abs(self.max_v):
            v_long = np.sign(self.v_long) * self.max_v
        self.acc_long = (v_long - self.v_long) / self.dt
        self.v_long = v_long

    def turn_wheel(self, direction):
        direction = np.sign(direction) * min(abs(direction), self.turn_inc)
        if self.turn_circle == 0:
            self.turn_circle = direction
            return
        turn_circle = direction + self.turn_circle
        turn_circle = np.sign(turn_circle) * min(self.max_turn_circle, abs(turn_circle))
        self.turn_circle = turn_circle

    def rotate(self, angle_increment_radians=0):
        self.direction += angle_increment_radians
        if self.direction < 0:
            self.direction += 2 * np.pi
        if abs(self.direction) > 2 * np.pi:
            self.direction = self.direction % (2 * np.pi)

        self._create_image_instance()

        previous_image_center = self.image.get_rect().center
        rotated_image = pygame.transform.rotate(self.image, np.rad2deg(self.direction))
        rotated_image_center = rotated_image.get_rect().center
        offset = [rotated_image_center[0] - previous_image_center[0],
                  rotated_image_center[1] - previous_image_center[1]]

        self.image = rotated_image
        self.image_offset = [self.image_offset[0] + offset[0], self.image_offset[1] + offset[1]]

    def update_offset(self, offset):
        self.draw_offset = offset
        self.draw_x = self.world_x * self.draw_scale + offset[0]
        self.draw_y = self.world_y * self.draw_scale + offset[1]

    def update_scale(self, scale):
        inc = scale / self.draw_scale
        self.pixel_to_metres_ratio *= inc
        self.draw_size = [self.actual_vehicle_size[0] * self.pixel_to_metres_ratio,
                          self.actual_vehicle_size[1] * self.pixel_to_metres_ratio]
        self.rotate()
        self.draw_scale = scale

    def _create_image_instance(self):
        self.original_image = pygame.image.load(self.image_path)
        self.original_image = pygame.transform.rotate(self.original_image, 180)
        self.image = pygame.transform.smoothscale(self.original_image, self.draw_size)
        self.image_offset = self.image.get_rect().center

    def is_left(self, p):
        self.vehicle_forward_point = [
            self.world_x + np.cos(self.direction),
            self.world_y - np.sin(self.direction)]
        return is_left([self.world_x, self.world_y], self.vehicle_forward_point, p)

    def pov_distances(self, point, angle):
        dist = np.sqrt((point[0] - self.world_x) ** 2 + (point[1] - self.world_y) ** 2)
        d_lat = dist * np.sin(angle)
        d_long = dist * np.cos(angle)
        return d_lat, d_long

    # endregion


class NormalVehicle(Vehicle):
    def __init__(self, starting_position=(0, 150), starting_direction_degrees=90, world=None):
        super().__init__(image_path=None, initial_position=starting_position,
                         starting_direction_degrees=starting_direction_degrees, world=world)

    def update(self, dt):
        self.accelerate(self.vel_inc)
        super().update(dt)
