import numpy as np
from model.Agent.Vehicle import Vehicle
import config

"""
Units in metric
Angles in radians, except for pygame rotations
    This goes from 0->PI, and from 0->-PI
"""


class RLEgoVehicle(Vehicle):
    def __init__(self, world=None):
        image_path = config.INPUT_IMAGE_FP + '/red_car_top_down.png'
        super().__init__(image_path=image_path, world=world)
        self.world = world


    # region overriding functions
    def send_message(self, string):
        super().send_message(string)

        if string == "u":
            self.accelerate(self.vel_inc)
        if string == "d":
            self.accelerate(-self.vel_inc)
        if string == "l":
            self.turn_wheel(np.sign(self.v_long) * self.turn_inc)
        if string == "r":
            self.turn_wheel(-np.sign(self.v_long) * self.turn_inc)

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
   # endregion

