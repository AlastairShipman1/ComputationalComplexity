import numpy as np
import pygame
import shapely.geometry
from visualisation.VisualisationUtils import Colours
# import Predictions
import config
from shapely.geometry import LineString


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
        self.acc_rate = 11
        self.friction_rate = 0.1
        self.turn_rate = 1
        self.max_v = 100
        self.original_image = Image(ego_vehicle)
        self.image = self.original_image.image
        self.offset = self.image.get_rect().center

        self.x = -self.offset[0] + 0
        self.y = -self.offset[1] + 150

    # endregion

    def move(self, dt=0):

        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # reduce vlong by friction- which is bigger when faster
        # dt is in milliseconds.
        self.x += self.v_long * dt / 1000 * np.cos(np.deg2rad(self.direction))
        self.y -= self.v_long * dt / 1000 * np.sin(np.deg2rad(self.direction))
        friction_scaler = 3
        if abs(self.v_long) < 10:
            friction_scaler = 0.5
        self.v_long -= np.sign(self.v_long) * self.friction_rate * friction_scaler

        if abs(self.v_long) < 5:
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

        self.offset = [self.offset[0] + offset[0], self.offset[1] + offset[1]]


class EgoVehicle(Vehicle):
    def __init__(self, world):
        super().__init__(ego_vehicle=True)
        self.world = world
        self.rays = []
        self.past_positions=[]
        # self.predicter = Predictions.ConstCurvaturePredicter()

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

        if len(self.past_positions)>2:
            self.past_positions=self.past_positions[-2:]
        self.past_positions.append([self.x, self.y])

        self.draw_sensor_rays(surface)
        self.draw_sensor_area(surface)
        # self.draw_predicted_motion(surface)

    def draw_predicted_motion(self, surface):
        predictions=self.predicter.make_predictions(self.past_positions)
        for pred in predictions:
            pygame.draw.circle(surface, Colours.GREEN, (pred[0], pred[1]), 4)

    def draw_sensor_area(self, surface):
        pygame.draw.aalines(surface, Colours.BLACK, True, self.rays)
        covered_area = shapely.geometry.Polygon(self.rays)
        max_area = shapely.geometry.Point((self.x, self.y)).buffer(200)
        diff = max_area.difference(covered_area)
        if isinstance(diff, shapely.geometry.Polygon):
            diff = shapely.geometry.MultiPolygon([diff])
        for geom in diff.geoms:
            pygame.draw.polygon(surface, Colours.BLACK, list(geom.exterior.coords))

    def draw_sensor_rays(self, surface):
        #  this needs to send out rays from a position on the car, across 360deg
        #  then, if these hit an obstacle, they should stop at the point of that obstacle.
        #  then we need to fill in the unknown areas behind these rays.

        # start with circular rays
        self.rays = []
        depth = 200
        angle = self.direction
        number_rays = 100
        for i in range(number_rays):
            # create a line between the car and the max length
            # cycle through all obstacles
            target_x = self.x + np.cos(np.deg2rad(angle)) * depth
            target_y = self.y - np.sin(np.deg2rad(angle)) * depth
            l = LineString([(self.x, self.y), (target_x, target_y)])
            for obs in self.world.obstacles:
                # if the line goes through the obstacle, then find the nearest point on the obstacle
                # and store this
                if l.intersects(obs.boundary):
                    p = l.intersection(obs.boundary)

                    curr_dist = (self.x - target_x) ** 2 + (self.y - target_y) ** 2
                    if isinstance(p, shapely.geometry.Point):
                        p = shapely.geometry.MultiPoint([p])

                    for point in list(p.geoms):
                        # pygame.draw.circle(surface, Colours.BLUE, (point.x, point.y), 4)
                        temp_target_x, temp_target_y = point.x, point.y
                        temp_dist = (self.x - temp_target_x) ** 2 + (self.y - temp_target_y) ** 2
                        if temp_dist < curr_dist:
                            target_x = temp_target_x
                            target_y = temp_target_y
                            curr_dist = temp_dist

            self.rays.append([target_x, target_y])
            pygame.draw.aaline(surface, (255, 255, 0), (self.x, self.y), (target_x, target_y))

            angle -= 360 / number_rays

