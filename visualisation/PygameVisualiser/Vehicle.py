import numpy as np
import pygame
import config


class Image(pygame.sprite.Sprite):
    def __init__(self, ego_vehicle=False):
        pygame.sprite.Sprite.__init__(self)
        image_path = config.input_image_file_path + '/blue_car_top_down.png'
        if ego_vehicle is None:
            image_path = config.input_image_file_path + '/red_car_top_down.png'

        self.image = pygame.image.load(image_path)
        self.image = pygame.transform.rotate(self.image, 180)

        scale = 0.2
        proposed_size = [self.image.get_width() * scale, self.image.get_height() * scale]
        self.image = pygame.transform.smoothscale(self.image, proposed_size)
        self.rect = self.image.get_rect()
        self.centre = self.rect.center


class Vehicle():

    # region Initialising functions
    def __init__(self, ego_vehicle=False):
        #### initialise position and direction ####
        self.v_long = 0
        self.v_lat = 0
        self.direction = 0
        self.acc_rate = 10
        self.friction_rate = 0
        self.turn_rate = 3
        self.max_v = 50
        self.original_image = Image(ego_vehicle)
        self.image = self.original_image.image
        self.offset = self.image.get_rect().center

        self.x = -self.offset[0]+100
        self.y = -self.offset[1]+100

    # endregion

    def move(self, dt=0):

        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # reduce vlong by friction
        # dt is in milliseconds.
        self.x += self.v_long * dt / 1000 * np.cos(np.deg2rad(self.direction))
        self.y -= self.v_long * dt / 1000 * np.sin(np.deg2rad(self.direction))
        if self.v_long > 0:
            self.v_long -= self.friction_rate
        else:
            self.v_long += self.friction_rate
        if abs(self.v_long) < 0.004:
            self.v_long = 0

    def update(self, dt):
        self.move(dt)

    def draw(self, surface):
        ##### draw agent on surface#########
        pos = (self.x - self.offset[0], self.y - self.offset[1])
        surface.blit(self.image, pos)

    def send_message(self, string):
        ...

    def rotate(self, angle):
        previous_image_center = self.image.get_rect().center
        rotated_image = pygame.transform.rotate(self.original_image.image, angle)
        rotated_image_center = rotated_image.get_rect().center
        offset = [rotated_image_center[0] - previous_image_center[0],
                  rotated_image_center[1] - previous_image_center[1]]
        self.image = rotated_image

        self.offset=[self.offset[0] + offset[0], self.offset[1] +offset[1]]


class EgoVehicle(Vehicle):
    def __init__(self):
        super().__init__(ego_vehicle=True)

    def send_message(self, string):
        if string == "u":
            v_long = self.v_long + self.acc_rate
            if v_long > abs(self.max_v):
                v_long = self.max_v
            self.v_long = v_long

        if string == "d":
            v_long = self.v_long - self.acc_rate
            if abs(v_long) > self.max_v:
                v_long = -self.max_v
            self.v_long = v_long

        if string == "l":
            self.direction += self.turn_rate
            self.rotate(self.direction)

        if string == "r":
            self.direction -= self.turn_rate
            self.rotate(self.direction)

    def draw(self, surface):
        ##### draw agent on surface#########
        pos = (self.x - self.offset[0], self.y - self.offset[1])
        surface.blit(self.image, pos)
