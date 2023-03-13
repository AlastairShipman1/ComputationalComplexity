import numpy as np
from shapely.geometry import CAP_STYLE, LineString

from visualisation.PygameVisualiser.Vehicle import Vehicle, Actor
import visualisation.PygameVisualiser.Predictions as Predictions
from visualisation.VisualisationUtils import Colours
import config
import pygame
import shapely


def is_left(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 0


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


        image_path = config.input_image_file_path + '/red_car_top_down.png'
        super().__init__(image_path=image_path)

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
        self.ray_length = 100  # in metres
        self.actors = [car]  # pedestrians, cars, bicycles, etc
        self.unknown_areas = None
        self.future_path_area = None
        self.closest_unknown_points = []
        self.latency = .1  # seconds
        self.is_limiting_speed = False
        self.limiting_point = None

        # bug finding for collision points
        self.collision_point_from_vehicle = None
        self.collision_point_from_actor = None
        self.rel_velocity = 0
        self.distance_to_collision = np.inf
        self.d_lat = 0
        self.d_long = 0
        self.world_worst_case_angle = 0
        self.rel_worst_case_angle = 0
        self.vehicle_forward_point = None
        self.actor_point = None

    # region drawing functions

    def draw(self, surface):
        self.draw_sensor_rays(surface)
        self.draw_sensor_area(surface)
        self.draw_predicted_motion(surface)
        # self.draw_closest_unknown_points(surface)
        self.draw_collision_points(surface)
        self.draw_collision_meta_data(surface)

        # actually draw the vehicle
        super().draw(surface)

    def draw_collision_meta_data(self, surface):
        if self.d_lat is None or self.d_long is None:
            return

        vehicle_d_long_line_world_end_point = [
            self.world_x + self.d_long * np.cos(self.direction),
            self.world_y - self.d_long * np.sin(self.direction)]

        vehicle_d_lat_line_world_end_point = [
            vehicle_d_long_line_world_end_point[0] + self.d_lat * np.cos(self.direction + np.pi / 2),
            vehicle_d_long_line_world_end_point[1] - self.d_lat * np.sin(self.direction + np.pi / 2)]

        d_long_draw_coords = self.convert_world_to_draw_coords(vehicle_d_long_line_world_end_point)
        d_lat_draw_coords = self.convert_world_to_draw_coords(vehicle_d_lat_line_world_end_point)

        pygame.draw.aaline(surface, Colours.GREEN, (self.draw_x, self.draw_y), d_long_draw_coords)
        pygame.draw.aaline(surface, Colours.GREEN, d_long_draw_coords, d_lat_draw_coords)

        # draw_forward_coords = self.convert_world_to_draw_coords(self.vehicle_forward_point)
        # pygame.draw.circle(surface, Colours.RED, draw_forward_coords, 10)

    def draw_collision_points(self, surface):
        if self.actor_point is None:
            return
        unknown_pt_draw_coords = self.convert_world_to_draw_coords(self.actor_point)
        pygame.draw.circle(surface, Colours.RED, unknown_pt_draw_coords, 10)

        collision_vehicle_draw_coords = self.convert_world_to_draw_coords(self.collision_point_from_vehicle)
        pygame.draw.circle(surface, Colours.RED, collision_vehicle_draw_coords, 5)
        pygame.draw.aaline(surface, Colours.RED, (self.draw_x, self.draw_y), collision_vehicle_draw_coords)

        collision_actor_draw_coords = self.convert_world_to_draw_coords(self.collision_point_from_actor)
        pygame.draw.circle(surface, Colours.BLUE, collision_actor_draw_coords, 5)
        pygame.draw.aaline(surface, Colours.BLUE, unknown_pt_draw_coords, collision_actor_draw_coords)

    def convert_world_to_draw_coords(self, coord):
        x = coord[0] * self.draw_scale + self.draw_offset[0]
        y = coord[1] * self.draw_scale + self.draw_offset[1]
        return x, y

    def draw_closest_unknown_points(self, surface):
        for point in self.closest_unknown_points:
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
        # for point in self.predictions_slow:
        #     pygame.draw.circle(surface, Colours.GREEN, self.convert_world_to_draw_coords(point), 2)
        if self.future_path_area is not None and not self.future_path_area.is_empty:
            coords = [self.convert_world_to_draw_coords([x, y])
                      for x, y in list(self.future_path_area.exterior.coords)]
            pygame.draw.polygon(surface, Colours.PURPLE, coords)

    # endregion

    # region overriding functions
    def send_message(self, string):
        super().send_message(string)
        if string == "u":
            self.accelerate(self.acc_rate)

        if string == "d":
            self.accelerate(-self.acc_rate)

        if string == "l":
            direction_increment = np.sign(self.v_long) * self.turn_rate
            self.rotate(direction_increment)

        if string == "r":
            direction_increment = np.sign(-self.v_long) * self.turn_rate
            self.rotate(direction_increment)

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
        self.check_speed_limit()

    # endregion


    # region class specific functions
    def check_speed_limit(self):
        if np.isfinite(self.time_to_col):
            # if the time to collision is less than the time it would take to brake to 0, plus latency
            # then we need to slow down
            brake_time = abs(self.v_long/self.max_acc_rate)
            if self.time_to_col < self.latency + brake_time:
                self.v_long -= np.sign(self.v_long)*self.acc_rate


    def angle_to_point(self, point):
        vector_1 = [(point[0] - self.world_x), (point[1] - self.world_y)]
        direction = (1 - np.sign(self.v_long)) * 0.5 * np.pi + self.direction
        vector_2 = [np.cos(direction), np.sin(-direction)]
        angle = angle_between(vector_1, vector_2)

        return angle

    def pov_distances(self, point, angle):
        dist = np.sqrt((point[0] - self.world_x) ** 2 + (point[1] - self.world_y) ** 2)
        d_lat = dist * np.sin(angle)
        d_long = dist * np.cos(angle)
        return d_lat, d_long

    def assess_closest_points(self):
        self.actor_point = None
        self.collision_point_from_actor = None
        self.collision_point_from_vehicle = None
        self.world_worst_case_angle = 0
        self.rel_worst_case_angle = 0
        self.d_lat = 0
        self.d_long = 0
        self.time_to_col = np.inf
        self.distance_to_collision = np.inf
        self.rel_velocity = 0

        # this is a point in front of the drivers eyes.
        # NOT in the direction of travel. this is important for the worst case angle
        self.vehicle_forward_point = [
            self.world_x + 10 * np.sign(self.v_long) * self.v_long * np.cos(self.direction),
            self.world_y - 10 * np.sign(self.v_long) * self.v_long * np.sin(self.direction)]

        for p in self.closest_unknown_points:
            # find the angle, lateral and longitudinal distances for this point
            # d_lat and angle are positive if the obstacle point is to the left of the vehicle, negative to the right
            # this is consistent if driving backwards (note that the forward direction is maintained in this case)

            angle = self.angle_to_point(p)
            d_lat, d_long = self.pov_distances(p, angle)
            if d_long == 0:
                continue

            # check if we're going clockwise, or counterclockwise
            if not is_left([self.world_x, self.world_y], self.vehicle_forward_point, p):
                angle = angle - np.pi / 2

            # this is used for ensuring we get the right quadrants
            obs_sign = np.sign(angle)
            vel_sign = np.sign(self.v_long)

            # limitations in the POV_distances function are addressed here
            d_lat *= obs_sign
            d_long *= vel_sign

            for actor in self.actors:
                # there are up to two angles. see powerpoint for derivation
                # we take the minimum angle, correct for cw/ccw and direction of vehicle speed
                # then make it relative to the world axis
                a_1, a_2 = self.find_worst_case_rel_angle(d_lat, d_long, actor.speed)

                rel_worst_case_angle = min(a_1, a_2) * obs_sign * vel_sign
                world_worst_case_angle = (rel_worst_case_angle + np.pi + self.direction) \
                                         % (2 * np.pi)  # to make it world direction, and not relative
                # this bit is kinematics and the cosine rule
                rel_dist = np.sqrt((p[0] - self.world_x) ** 2 + (p[1] - self.world_y) ** 2)
                rel_vel = np.sqrt(actor.speed ** 2 + self.v_long ** 2 - \
                                  (2 * actor.speed * vel_sign * self.v_long * np.cos(np.pi - rel_worst_case_angle)))

                time_to_col = rel_dist / rel_vel

                if time_to_col< self.time_to_col:
                    self.time_to_col=time_to_col
                    vehicle_collision_point = [
                        self.world_x + time_to_col * self.v_long * np.cos(self.direction),
                        self.world_y - time_to_col * self.v_long * np.sin(self.direction)]

                    actor_collision_point = [
                        p[0] + time_to_col * actor.speed * vel_sign * np.cos(world_worst_case_angle),
                        p[1] - time_to_col * actor.speed * vel_sign * np.sin(world_worst_case_angle)]

                    self.actor_point = p
                    self.collision_point_from_actor = actor_collision_point
                    self.collision_point_from_vehicle = vehicle_collision_point
                    self.distance_to_collision = rel_dist
                    self.rel_velocity = rel_vel
                    self.world_worst_case_angle = world_worst_case_angle
                    self.rel_worst_case_angle = rel_worst_case_angle
                    self.d_lat = d_lat
                    self.d_long = d_long



    def calculate_closest_unknown_points(self):

        self.closest_unknown_points = []
        closest_dist = np.inf
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
                #
                if abs(angle) > np.pi / 2:
                    # pygame.draw.circle(surface, Colours.RED, (point.x, point.y), 5)
                    continue
                dist = (pt[0] - self.world_x) ** 2 + (pt[1] - self.world_y) ** 2
                self.closest_unknown_points.append(pt)

    def calculate_unknown_areas(self):
        covered_area = shapely.geometry.Polygon(self.ray_endpoints)
        max_area = shapely.geometry.Point((self.world_x, self.world_y)).buffer(self.ray_length)
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

        angle = self.direction
        number_rays = 100
        for i in range(number_rays):
            # create a line between the car and the max length
            # cycle through all obstacles
            target_x = self.world_x + np.cos(angle) * self.ray_length
            target_y = self.world_y - np.sin(angle) * self.ray_length
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
            angle -= (np.pi * 2) / number_rays

    def find_worst_case_rel_angle(self, d_lat, d_long, v_actor):

        x = abs(d_lat / d_long)
        y = abs(self.v_long / v_actor)

        # see powerpoint for a derivation of these values
        a = (x ** 2) + 1
        b = 2 * (x ** 2) * y
        c = ((x ** 2) * (y ** 2)) - 1

        if ((b ** 2) - (4 * a * c)) < 0:
            return np.nan, np.nan

        angle_1, angle_2 = np.inf, np.inf
        A = (-b + np.sqrt((b ** 2) - (4 * a * c))) / (2 * a)
        B = (-b - np.sqrt((b ** 2) - (4 * a * c))) / (2 * a)

        if abs(A) <= 1:
            angle_1 = np.arccos(A)
        if abs(B) <= 1:
            angle_2 = np.arccos(B)
        if not np.isfinite(angle_1) and not np.isfinite(angle_2):
            return np.nan, np.nan

        return angle_1, angle_2

    # endregion
