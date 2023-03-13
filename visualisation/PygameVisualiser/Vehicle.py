import numpy as np
import pygame

import shapely.geometry
from shapely.ops import nearest_points
from shapely.geometry.base import CAP_STYLE
from visualisation.VisualisationUtils import Colours
from visualisation.PygameVisualiser import Predictions, Obstacle
import config
from shapely.geometry import LineString

""" 
File for agents
Units in metric
angular units in radians, varying 0->2pi
"""


class Actor:
    def __init__(self):
        self.speed = 5


class Vehicle:

    def __init__(self, image_path=None, initial_position=(0, 150), starting_direction_degrees=0):

        # simulation variables
        super().__init__()
        self.v_long = 0
        self.v_lat = 0
        self.acc_long = 0
        self.direction = np.deg2rad(starting_direction_degrees)
        self.acc_rate = 1
        self.max_acc_rate = 5
        self.friction_rate = 0
        self.turn_rate = 0.05
        self.max_v = 30  # 30 is approximately 70mph
        self.dt = np.inf

        # drawing/visualisation variables
        self.image_path = image_path
        if image_path is None:
            self.image_path = config.input_image_file_path + '/blue_car_top_down.png'

        self.actual_vehicle_size = [6, 3]
        self.pixel_to_size_ratio = 1
        self.draw_size = [self.actual_vehicle_size[0] * self.pixel_to_size_ratio,
                          self.actual_vehicle_size[1] * self.pixel_to_size_ratio]

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

    # region utils
    def move(self):
        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # reduce vlong by friction- which is bigger when faster
        # dt is in milliseconds.
        x_addition = self.v_long * self.dt / 1000 * np.cos(self.direction)
        self.world_x += x_addition
        y_addition = self.v_long * self.dt / 1000 * np.sin(self.direction)
        self.world_y -= y_addition
        friction_scaler = 1
        if abs(self.v_long) < 15:
            friction_scaler = 1.5
        self.v_long -= np.sign(self.v_long) * self.friction_rate * friction_scaler

        if abs(self.v_long) < 1:
            self.v_long = 0

    def update(self, dt):
        self.dt = dt
        self.move()
        self.update_offset(self.draw_offset)

    def draw(self, surface):
        ##### draw agent on surface#########
        pos = (self.draw_x - self.image_offset[0], self.draw_y - self.image_offset[1])
        surface.blit(self.image, pos)

    def send_message(self, string):
        ...

    def accelerate(self, amount):
        v_long = self.v_long + amount
        if abs(v_long) > abs(self.max_v):
            v_long = np.sign(self.v_long) * self.max_v
        self.acc_long = (v_long - self.v_long) / self.dt
        self.v_long = v_long

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

    # endregion
    def update_offset(self, offset):
        self.draw_offset = offset
        self.draw_x = self.world_x * self.draw_scale + offset[0]
        self.draw_y = self.world_y * self.draw_scale + offset[1]

    def update_scale(self, scale):
        inc = scale / self.draw_scale
        self.pixel_to_size_ratio *= inc
        self.draw_size = [self.actual_vehicle_size[0] * self.pixel_to_size_ratio,
                          self.actual_vehicle_size[1] * self.pixel_to_size_ratio]
        self.rotate()
        self.draw_scale = scale

    def _create_image_instance(self):
        self.original_image = pygame.image.load(self.image_path)
        self.original_image = pygame.transform.rotate(self.original_image, 180)
        self.image = pygame.transform.smoothscale(self.original_image, self.draw_size)
        self.image_offset = self.image.get_rect().center


class NormalVehicle(Vehicle):
    def __init__(self, starting_position=(0, 150), starting_direction_degrees=90):
        super().__init__(image_path=None, initial_position=starting_position,
                         starting_direction_degrees=starting_direction_degrees)

    def update(self, dt):
        self.accelerate(self.acc_rate)
        super().update(dt)
