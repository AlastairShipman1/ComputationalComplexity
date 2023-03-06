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


class Actor:
    def __init__(self):
        self.speed = 1


class Vehicle:

    def __init__(self, ego_vehicle=False, initial_position=(0, 150), starting_direction=0):
        # drawing/visualisation variables
        self.image_path = config.input_image_file_path + '/blue_car_top_down.png'
        if ego_vehicle:
            self.image_path = config.input_image_file_path + '/red_car_top_down.png'

        self.actual_vehicle_size = [6, 3]
        self.pixel_to_size_ratio = 1
        self.draw_size = [self.actual_vehicle_size[0] * self.pixel_to_size_ratio,
                          self.actual_vehicle_size[1] * self.pixel_to_size_ratio]

        self.original_image = pygame.image.load(self.image_path)
        self.original_image = pygame.transform.rotate(self.original_image, 180)
        self.image = pygame.transform.smoothscale(self.original_image, self.draw_size)
        self.rotate(starting_direction)

        self.image_offset = self.image.get_rect().center
        self.draw_offset = 0, 0
        self.draw_scale = 1

        # game variables
        self.v_long = 0
        self.v_lat = 0
        self.acc_long = 0
        self.direction = starting_direction
        self.acc_rate = 5
        self.friction_rate = 1
        self.turn_rate = 1
        self.max_v = 10  # 30 is approximately 70mph
        self.world_x = -self.image_offset[0] + initial_position[0]
        self.world_y = -self.image_offset[1] + initial_position[1]
        self.draw_x = self.world_x
        self.draw_y = self.world_y
        self.dt = np.inf

    # region utils
    def move(self):
        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # reduce vlong by friction- which is bigger when faster
        # dt is in milliseconds.
        x_addition = self.v_long * self.dt / 1000 * np.cos(np.deg2rad(self.direction))
        self.world_x += x_addition
        y_addition = self.v_long * self.dt / 1000 * np.sin(np.deg2rad(self.direction))
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
        # image_x = self.x * self.draw_scale * self.monitor_size[0] \
        #           - self.image.get_width() / 2 + self.draw_offset[0]
        # image_y = self.y * self.draw_scale * self.monitor_size[1] \
        #           - self.image.get_height() / 2 + self.draw_offset[1]
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

    def rotate(self, angle):
        self._create_image_instance()
        previous_image_center = self.image.get_rect().center
        rotated_image = pygame.transform.rotate(self.image, angle)

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
        self.rotate(self.direction)
        self.draw_scale = scale

    def _create_image_instance(self):
        self.original_image = pygame.image.load(self.image_path)
        self.original_image = pygame.transform.rotate(self.original_image, 180)
        self.image = pygame.transform.smoothscale(self.original_image, self.draw_size)
        self.image_offset = self.image.get_rect().center


class EgoVehicle(Vehicle):
    def __init__(self, world):
        super().__init__(ego_vehicle=True)

        self.time_to_col = np.inf
        self.predictions_fast = []
        self.predictions_slow = []
        self.world = world
        self.ray_endpoints = []
        self.past_positions = []
        self.past_positions_length = 31  # should be an odd number
        self.predicter_slow = Predictions.ConstCurvaturePredicter(self, self.past_positions_length)
        self.predicter_fast = Predictions.ConstCurvaturePredicter(self, 5)

        car = Actor()
        car.speed = 10

        self.ray_length = 200  # in metres
        self.actors = [car]  # pedestrians, cars, bicycles, etc
        self.unknown_areas = None
        self.future_path_area = None
        self.closest_unknown_points = []
        self.latency = 1  # seconds
        self.is_limiting_speed = False
        self.limiting_point = None

        self.collision_points_from_vehicle = []
        self.collision_points_from_actor = []

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
        self.draw_collision_points(surface)

        # actually draw the vehicle
        super().draw(surface)

    def draw_collision_points(self, surface):
        for i, unknown_pt in enumerate(self.closest_unknown_points):
            unknown_pt_draw_coords = self.convert_world_to_draw_coords(unknown_pt)
            pygame.draw.circle(surface, Colours.RED, unknown_pt_draw_coords, 10)

            collision_vehicle_draw_coords = self.convert_world_to_draw_coords(self.collision_points_from_vehicle[i])
            pygame.draw.circle(surface, Colours.RED, collision_vehicle_draw_coords, 5)
            pygame.draw.aaline(surface, Colours.RED, (self.draw_x, self.draw_y), collision_vehicle_draw_coords)

            collision_actor_draw_coords = self.convert_world_to_draw_coords(self.collision_points_from_actor[i])
            pygame.draw.circle(surface, Colours.BLUE, collision_actor_draw_coords, 5)
            pygame.draw.aaline(surface, Colours.BLUE, unknown_pt_draw_coords, collision_actor_draw_coords)

    def convert_world_to_draw_coords(self, coord):
        x = coord[0] * self.draw_scale + self.draw_offset[0]
        y = coord[1] * self.draw_scale + self.draw_offset[1]
        return x, y

    def draw_closest_unknown_points(self, surface):
        for point in self.closest_unknown_points:
            angle = self.angle_to_point(point)

            if angle > 90:
                # pygame.draw.circle(surface, Colours.RED, (point.x, point.y), 5)
                continue
            pt = self.convert_world_to_draw_coords([point[0], point[1]])

            pygame.draw.circle(surface, Colours.BLUE, pt, 5)

    def draw_sensor_rays(self, surface):
        ray_points = [self.convert_world_to_draw_coords([x, y]) for x, y in self.ray_endpoints]

        for rp in ray_points:
            pygame.draw.aaline(surface, (255, 255, 0), (self.draw_x, self.draw_y), rp)

    def draw_sensor_area(self, surface):
        ray_points = [self.convert_world_to_draw_coords([x, y])
                      for x, y in self.ray_endpoints]

        pygame.draw.aalines(surface, Colours.BLACK, True, ray_points)
        for i, geom in enumerate(self.unknown_areas.geoms):
            coords = [self.convert_world_to_draw_coords([x, y])
                      for x, y in list(geom.exterior.coords)]
            pygame.draw.polygon(surface, Colours.BLACK, coords)
            pygame.draw.polygon(surface, Colours.WHITE, coords, 1)
            font = pygame.font.SysFont(None, 24)
            img = font.render(str(i), True, Colours.WHITE)
            x = geom.centroid.coords.xy
            surface.blit(img, self.convert_world_to_draw_coords([x[0][0], x[1][0]]))

    def draw_predicted_motion(self, surface):
        for point in self.predictions_slow:
            pygame.draw.circle(surface, Colours.GREEN, self.convert_world_to_draw_coords(point), 2)
        if self.future_path_area is not None and not self.future_path_area.is_empty:
            coords = [self.convert_world_to_draw_coords([x, y])
                      for x, y in list(self.future_path_area.exterior.coords)]
            pygame.draw.polygon(surface, Colours.PURPLE, coords)

    def update(self, dt):
        super().update(dt)

        self.past_positions.append([self.world_x, self.world_y, self.dt])
        if len(self.past_positions) > self.past_positions_length - 1:
            self.past_positions = self.past_positions[-self.past_positions_length:]

        self.shoot_rays()
        self.calculate_unknown_areas()
        self.calculate_predicted_motion()
        self.calculate_closest_unknown_points()

        # now we need to calculate the maximum speed for this particular scenario.
        self.assess_closest_points()

    def angle_to_point(self, point):
        vector_1 = [(point[0] - self.world_x) * np.sign(self.v_long), (point[1] - self.world_y) * np.sign(-self.v_long)]
        vector_2 = [np.cos(np.deg2rad(self.direction)), np.sin(np.deg2rad(self.direction))]
        angle = np.rad2deg(angle_between(vector_1, vector_2))
        return angle

    def pov_distances(self, point, angle):
        dist = np.sqrt((point[1] - self.world_x) ** 2 + (point[1] - self.world_y) ** 2)
        d_lat = dist * np.sin(np.deg2rad(angle))
        d_long = dist * np.cos(np.deg2rad(angle))
        return d_lat, d_long

    def assess_closest_points(self):
        self.collision_points_from_actor = []
        self.collision_points_from_vehicle = []
        self.is_limiting_speed = False
        self.time_to_col = np.inf
        # if self.v_long == 0:
        #     return

        for p in self.closest_unknown_points:

            # if it is behind the vehicle's direction of travel, ignore it.
            angle = self.angle_to_point(p)
            # find the lateral and longitudinal distance for this point
            d_lat, d_long = self.pov_distances(p, angle)

            for actor in self.actors:
                worst_case_rel_angle = self.find_worst_case_actor_rel_direction(d_lat, d_long, actor.speed)
                rel_dist_squared = d_lat ** 2 + d_long ** 2
                rel_vel_squared = actor.speed ** 2 + self.v_long ** 2 \
                                 - 2 * actor.speed * self.v_long * np.cos(np.deg2rad(180 - worst_case_rel_angle))
                time_to_col = np.sqrt(rel_dist_squared / rel_vel_squared)
                world_worst_case_angle = worst_case_rel_angle + 180 + self.direction  # to make it world direction, and not relative

                self.collision_points_from_vehicle.append([
                    self.world_x + time_to_col * self.v_long * np.cos(np.deg2rad(self.direction)),
                    self.world_y - time_to_col * self.v_long * np.sin(np.deg2rad(self.direction))])

                self.collision_points_from_actor.append([
                    p[0] + time_to_col * actor.speed * np.cos(np.deg2rad(world_worst_case_angle)),
                        p[1] - time_to_col * actor.speed * np.sin(np.deg2rad((world_worst_case_angle)))])

                # max_v_long = self.acc_rate * (time_to_col - self.latency)
                # if max_v_long < abs(self.v_long):
                #     self.is_limiting_speed = True
                #     self.limiting_point = p
                #     self.v_long -= np.sign(self.v_long) * self.acc_rate

                # if max speed is lower than actual speed, need to limit this

    def calculate_closest_unknown_points(self):

        self.closest_unknown_points = []
        closest_dist=np.inf
        if self.unknown_areas is None:
            return
        if self.future_path_area is None:
            return

        for g in self.unknown_areas.geoms:
            if self.future_path_area.is_empty:
                return
            # p = shapely.geometry.Point(self.world_x, self.world_y)
            # closest_points = nearest_points(g, p)
            pts = g.exterior.coords[:-1]
            for pt in pts:
                angle = self.angle_to_point(pt)


                if angle > 90:
                    # pygame.draw.circle(surface, Colours.RED, (point.x, point.y), 5)
                    continue
                dist = (pt[0] - self.world_x) ** 2 + (pt[1] - self.world_y) ** 2
                if dist<closest_dist:
                    self.closest_unknown_points=[]
                    self.closest_unknown_points.append(pt)
                    closest_dist=dist

    def calculate_unknown_areas(self):
        covered_area = shapely.geometry.Polygon(self.ray_endpoints)
        max_area = shapely.geometry.Point((self.world_x, self.world_y)).buffer(200)
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
        self.ray_endpoints = []
        depth = 200
        angle = self.direction
        number_rays = 100
        for i in range(number_rays):
            # create a line between the car and the max length
            # cycle through all obstacles
            target_x = self.world_x + np.cos(np.deg2rad(angle)) * depth
            target_y = self.world_y - np.sin(np.deg2rad(angle)) * depth
            l = LineString([(self.world_x, self.world_y), (target_x, target_y)])
            for obs in self.world.obstacles.geoms:
                # if the line goes through the obstacle, then find the nearest point on the obstacle
                # and store this
                if l.intersects(obs.boundary):
                    p = l.intersection(obs.boundary)

                    curr_dist = (self.world_x - target_x) ** 2 + (self.world_y - target_y) ** 2
                    if isinstance(p, shapely.geometry.Point):
                        p = shapely.geometry.MultiPoint([p])

                    for point in list(p.geoms):
                        temp_target_x, temp_target_y = point.x, point.y
                        temp_dist = (self.world_x - temp_target_x) ** 2 + (self.world_y - temp_target_y) ** 2
                        if temp_dist < curr_dist:
                            target_x = temp_target_x
                            target_y = temp_target_y
                            curr_dist = temp_dist

            self.ray_endpoints.append([target_x, target_y])
            angle -= 360 / number_rays

    def find_worst_case_actor_rel_direction(self, d_lat, d_long, v_actor):

        # can the actor actually reach us?
        # if so, can lead to a collision, assess the maximum speed
        # if abs(d_lat / v_actor) < abs(d_long / self.v_long):
        #     return np.inf

        # evaluate the worst case angle for an actor to approach the vehicle
        angle_1 = self.find_worst_case_rel_angle(d_lat, d_long, v_actor)
        angle = np.rad2deg(angle_1)

        return angle

    def find_worst_case_rel_angle(self, d_lat, d_long, v_actor):
        x = d_lat / d_long
        y = self.v_long / v_actor

        # see powerpoint for a derivation of these values
        a = (x ** 2) + 1
        b = 2 * (x ** 2) * y
        c = ((x ** 2) * (y ** 2)) - 1

        if ((b ** 2) - (4 * a * c)) < 0:
            return np.nan

        angle_1, angle_2 = np.inf, np.inf
        A = (-b + np.sqrt((b ** 2) - (4 * a * c))) / (2 * a)
        B = (-b - np.sqrt((b ** 2) - (4 * a * c))) / (2 * a)

        if abs(A) <= 1:
            angle_1 = np.arccos(A)
        if abs(B) <= 1:
            angle_2 = np.arccos(B)

        if not np.isfinite(angle_1) and not np.isfinite(angle_2):
            return np.nan

        if angle_1 < 0:
            angle_1 = np.inf
        if angle_2 < 0:
            angle_2 = np.inf

        return np.nanmin([angle_2, angle_1])


class NormalVehicle(Vehicle):
    def __init__(self, starting_position=(0, 150), starting_direction=90):
        super(NormalVehicle, self).__init__(initial_position=starting_position, starting_direction=starting_direction)

    def update(self, dt):
        self.accelerate(self.acc_rate)
        super().update(dt)
