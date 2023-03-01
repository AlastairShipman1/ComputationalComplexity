import numpy as np
import pygame
import shapely.geometry
from shapely.ops import nearest_points
from shapely.geometry.base import CAP_STYLE
from visualisation.VisualisationUtils import Colours
from visualisation.PygameVisualiser import Predictions
import config
from shapely.geometry import LineString

""" 
File for agents
Units in metric
"""


class Actor:
    def __init__(self):
        self.speed = 1


class Image(pygame.sprite.Sprite):
    def __init__(self, ego_vehicle=False):
        pygame.sprite.Sprite.__init__(self)
        image_path = config.input_image_file_path + '/blue_car_top_down.png'
        if ego_vehicle:
            image_path = config.input_image_file_path + '/red_car_top_down.png'

        self.image = pygame.image.load(image_path)
        self.image = pygame.transform.rotate(self.image, 180)

        proposed_size = [30, 15]  # [6, 3] #6m by 3m.. need to make this resizable
        self.image = pygame.transform.smoothscale(self.image, proposed_size)
        self.rect = self.image.get_rect()
        self.centre = self.rect.center


class Vehicle:

    def __init__(self, ego_vehicle=False, initial_position=(0, 150)):
        self.v_long = 0
        self.v_lat = 0
        self.acc_long = 0
        self.direction = 0
        self.acc_rate = 5
        self.friction_rate = 1
        self.turn_rate = 1
        self.max_v = 30  # approximately 70mph
        self.original_image = Image(ego_vehicle)
        self.image = self.original_image.image
        self.offset = self.image.get_rect().center

        self.x = -self.offset[0] + initial_position[0]
        self.y = -self.offset[1] + initial_position[1]
        self.dt = np.inf

    # region utils
    def move(self):
        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # reduce vlong by friction- which is bigger when faster
        # dt is in milliseconds.
        x_addition = self.v_long * self.dt / 1000 * np.cos(np.deg2rad(self.direction))
        self.x += x_addition
        y_addition = self.v_long * self.dt / 1000 * np.sin(np.deg2rad(self.direction))
        self.y -= y_addition
        friction_scaler = 1
        if abs(self.v_long) < 15:
            friction_scaler = 1.5
        self.v_long -= np.sign(self.v_long) * self.friction_rate * friction_scaler

        if abs(self.v_long) < 1:
            self.v_long = 0

    def update(self, dt):
        self.dt = dt
        self.move()

    def draw(self, surface):
        ##### draw agent on surface#########
        pos = (self.x - self.offset[0], self.y - self.offset[1])
        surface.blit(self.image, pos)

    def send_message(self, string):
        ...

    def accelerate(self, amount):
        v_long = self.v_long + amount
        if abs(v_long) > abs(self.max_v):
            v_long = np.sign(self.v_long) * self.max_v
        self.acc_long = (v_long - self.v_long) / self.dt
        self.v_long = v_long

    def rotate(self, angle):
        previous_image_center = self.image.get_rect().center
        rotated_image = pygame.transform.rotate(self.original_image.image, angle)
        rotated_image_center = rotated_image.get_rect().center
        offset = [rotated_image_center[0] - previous_image_center[0],
                  rotated_image_center[1] - previous_image_center[1]]
        self.image = rotated_image

        self.offset = [self.offset[0] + offset[0], self.offset[1] + offset[1]]
    # endregion


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    mag = np.linalg.norm(vector)
    if mag == 0:
        return [0, 0]
    return vector / mag


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


class EgoVehicle(Vehicle):
    def __init__(self, world):
        super().__init__(ego_vehicle=True)
        self.predictions_fast = []
        self.predictions_slow = []
        self.world = world
        self.rays = []
        self.past_positions = []
        self.past_positions_length = 31  # should be an odd number
        self.predicter_slow = Predictions.ConstCurvaturePredicter(self, self.past_positions_length)
        self.predicter_fast = Predictions.ConstCurvaturePredicter(self, 5)

        ped = Actor()
        ped.speed = 1
        car = Actor()
        car.speed = 100

        self.actors = [ped, car]  # pedestrians, cars, bicycles, etc
        self.unknown_areas = None
        self.future_path_area = None
        self.closest_unknown_points = []
        self.latency = 1  # seconds
        self.is_limiting_speed = False
        self.limiting_point = None

    def send_message(self, string):
        super().send_message(string)
        if string == "u":
            self.accelerate(self.acc_rate)

        if string == "d":
            self.accelerate(-self.acc_rate)

        if string == "l":
            self.direction += np.sign(self.v_long) * self.turn_rate
            self.rotate(self.direction)

        if string == "r":
            self.direction -= np.sign(self.v_long) * self.turn_rate
            self.rotate(self.direction)

    def draw(self, surface):
        self.draw_sensor_rays(surface)
        self.draw_sensor_area(surface)
        self.draw_predicted_motion(surface)
        self.draw_closest_unknown_points(surface)
        if self.is_limiting_speed and self.limiting_point is not None:
            pygame.draw.circle(surface, Colours.GREEN, (self.limiting_point.x, self.limiting_point.y), 10)
        pos = (self.x - self.offset[0], self.y - self.offset[1])
        surface.blit(self.image, pos)

    def draw_closest_unknown_points(self, surface):
        for point in self.closest_unknown_points:
            angle = self.angle_to_point(point)

            if angle > 90:
                # pygame.draw.circle(surface, Colours.RED, (point.x, point.y), 5)
                continue
            pygame.draw.circle(surface, Colours.BLUE, (point.x, point.y), 5)

    def draw_sensor_rays(self, surface):
        for ray in self.rays:
            pygame.draw.aaline(surface, (255, 255, 0), (self.x, self.y), ray)

    def draw_sensor_area(self, surface):
        pygame.draw.aalines(surface, Colours.BLACK, True, self.rays)
        for i, geom in enumerate(self.unknown_areas.geoms):
            pygame.draw.polygon(surface, Colours.BLACK, list(geom.exterior.coords))
            pygame.draw.polygon(surface, Colours.WHITE, list(geom.exterior.coords), 1)
            font = pygame.font.SysFont(None, 24)
            img = font.render(str(i), True, Colours.WHITE)
            x = geom.centroid.coords.xy
            surface.blit(img, (x[0][0], x[1][0]))

    def draw_predicted_motion(self, surface):
        for point in self.predictions_slow:
            pygame.draw.circle(surface, Colours.GREEN, (point[0], point[1]), 2)
        if self.future_path_area is not None and not self.future_path_area.is_empty:
            pygame.draw.polygon(surface, Colours.PURPLE, list(self.future_path_area.exterior.coords))

    def update(self, dt):
        super().update(dt)

        self.past_positions.append([self.x, self.y, self.dt])
        if len(self.past_positions) > self.past_positions_length - 1:
            self.past_positions = self.past_positions[-self.past_positions_length:]

        self.shoot_rays()
        self.calculate_unknown_areas()
        self.calculate_predicted_motion()
        self.calculate_closest_unknown_points()

        # now we need to calculate the maximum speed for this particular scenario.
        self.assess_closest_points()

    def angle_to_point(self, point):
        vector_1 = [(point.x - self.x) * np.sign(self.v_long), (point.y - self.y) * np.sign(-self.v_long)]
        vector_2 = [np.cos(np.deg2rad(self.direction)), np.sin(np.deg2rad(self.direction))]
        angle = np.rad2deg(angle_between(vector_1, vector_2))
        return angle

    def pov_distances(self, point, angle):
        dist = np.sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)
        d_lat = dist * np.sin(np.deg2rad(angle))
        d_long = dist * np.cos(np.deg2rad(angle))
        return d_lat, d_long

    def assess_closest_points(self):
        self.is_limiting_speed=False
        if self.v_long == 0:
            return

        for p in self.closest_unknown_points:

            # if it is behind the vehicle's direction of travel, ignore it.
            angle = self.angle_to_point(p)

            if angle > 90:
                continue

            # find the lateral and longitudinal distance for this point
            d_lat, d_long = self.pov_distances(p, angle)

            for actor in self.actors:
                max_v = self.assess_max_speed(d_lat, d_long, actor.speed)
                if max_v < abs(self.max_v):
                    self.is_limiting_speed = True
                    self.limiting_point = p
                    if abs(self.v_long) > max_v:
                        self.v_long = np.sign(self.v_long) * max_v
                    # if current speed is higher than maximum speed, need to limit this

    def calculate_closest_unknown_points(self):

        self.closest_unknown_points = []
        if self.unknown_areas is None:
            return
        if self.future_path_area is None:
            return

        for g in self.unknown_areas.geoms:
            if self.future_path_area.is_empty:
                return

            closest_points = nearest_points(g, self.future_path_area)
            self.closest_unknown_points.append(closest_points[0])

    def calculate_unknown_areas(self):
        covered_area = shapely.geometry.Polygon(self.rays)
        max_area = shapely.geometry.Point((self.x, self.y)).buffer(200)
        if max_area.is_empty:
            self.unknown_areas = None
            return
        self.unknown_areas = max_area.difference(covered_area)
        self.unknown_areas = self.unknown_areas.difference(self.world.obstacles)

        if isinstance(self.unknown_areas, shapely.geometry.Polygon):
            self.unknown_areas = shapely.geometry.MultiPolygon([self.unknown_areas])

        simple_geoms = []
        d = 1  # distance

        for g in self.unknown_areas.geoms:

            # now get rid of any long sharp thin areas, by taking a negative buffer and intersection
            # this might be a bit of an issue?
            # g = g.buffer(-d).intersection(g).simplify(d)
            if g.area > 2:
                if isinstance(g, shapely.geometry.Polygon):
                    g = shapely.geometry.MultiPolygon([g])
                for geom in g.geoms:
                    simple_geoms.append(geom)

        self.unknown_areas = shapely.geometry.MultiPolygon(simple_geoms)

    def calculate_predicted_motion(self):
        if len(self.past_positions) != self.past_positions_length:
            return
        self.predictions_slow = self.predicter_slow.make_predictions(self.past_positions)
        self.predictions_fast = self.predicter_fast.make_predictions(self.past_positions)
        self.predictions_slow.extend(self.predictions_fast[::-1])

        self.future_path_area = shapely.geometry.Polygon(self.predictions_slow).buffer(1, cap_style=CAP_STYLE.flat)
        if not isinstance(self.future_path_area, shapely.geometry.Polygon):
            self.future_path_area = None

    def shoot_rays(self):
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
            for obs in self.world.obstacles.geoms:
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
            angle -= 360 / number_rays

    def assess_max_speed(self, d_lat, d_long, v_actor):

        # can the actor actually reach us?
        # if so, can lead to a collision, assess the maximum speed
        if abs(d_lat / v_actor) < abs(d_long / self.v_long):
            return np.inf

        # evaluate the worst case angle for an actor to approach the vehicle
        angle_1 = self.find_worst_case_rel_angle(d_lat, d_long, v_actor)
        angle = np.rad2deg(angle_1)
        square_dist_to_collision = d_lat ** 2 + d_long ** 2
        square_rel_vel = v_actor ** 2 + self.v_long ** 2 - 2 * v_actor * self.v_long * np.cos(np.deg2rad(180 - angle))
        time_to_col = np.sqrt(square_dist_to_collision / square_rel_vel)

        max_v_long = self.acc_rate * (time_to_col - self.latency)
        return max_v_long

    def find_worst_case_rel_angle(self, d_lat, d_long, v_actor):
        x = d_lat / d_long
        v_ratio = self.v_long / v_actor

        # see powerpoint for a derivation of these values
        a = 2
        b = 2 * v_ratio
        c = (x * v_ratio) ** 2 - 1

        A = -b + np.sqrt(b ** 2 - 4 * a * c)
        angle_1 = np.arccos(A)
        B = -b - np.sqrt(b ** 2 - 4 * a * c)
        angle_2 = np.arccos(B)

        return np.nanmin([angle_2, angle_1])


class NormalVehicle(Vehicle):
    def __init__(self, starting_position=(0, 150), starting_direction=90):
        super(NormalVehicle, self).__init__()
        self.x = starting_position[0]
        self.y = starting_position[1]

        self.waypoints = []
        self.current_target = None
        self.rotate(starting_direction)
        self.direction = starting_direction

    def update(self, dt):
        self.dt = dt
        self.accelerate(self.acc_rate)
        self.move()
