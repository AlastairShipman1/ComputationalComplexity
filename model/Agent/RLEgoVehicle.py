import numpy as np
import pygame

from model.Agent.Vehicle import Vehicle
import config
from visualisation.VisualisationUtils import Colours

"""
Units in metric
Angles in radians, except for pygame rotations
    This goes from 0->PI, and from 0->-PI
"""


class RLEgoVehicle(Vehicle):
    def __init__(self, world=None):
        image_path = config.INPUT_IMAGE_FP + '/blue_car_top_down.png'
        super().__init__(image_path=image_path, world=world)
        self.world = world
        self.set_goal([0, 0])

    def set_goal(self, goal):
        self.goal=goal

    def set_position(self, position):
        self.world_x = position[0]
        self.world_y = position[1]
    # region overriding functions
    def send_message(self, string):
        super().send_message(string)

        if string == "u":
            self.accelerate(self.vel_inc)
        if string == "d":
            self.accelerate(-self.vel_inc)
        if string == "l":
            self.turn_wheel(np.sign(self.v_long) * self.turn_rate_inc)
        if string == "r":
            self.turn_wheel(-np.sign(self.v_long) * self.turn_rate_inc)

    # endregion

    # region class specific functions
    def parse_control(self, action):
        throttle = action[0]
        steer = action[1]

        if throttle == 2:
            self.send_message('u')
        if throttle == 0:
            self.send_message('d')
        if steer == 2:
            self.send_message('r')
        if steer == 0:
            self.send_message('l')

    def draw(self, surface):
        super(RLEgoVehicle, self).draw(surface)
        goal_coords = self.convert_world_to_draw_coords(self.goal)
        if goal_coords[0]>0:
            pygame.draw.circle(surface, Colours.RED, goal_coords, 10)
   # endregion

