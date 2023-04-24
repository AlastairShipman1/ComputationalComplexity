import numpy as np
from shapely.geometry import CAP_STYLE, LineString
import pygame
import shapely

from model.Agent.Vehicle import Vehicle, Actor
import model.Agent.Predictions as Predictions
from model.ModelUtils import angle_between, is_left
from visualisation.VisualisationUtils import Colours
import config

"""
Units in metric
Angles in radians, except for pygame rotations
    This goes from 0->PI, and from 0->-PI
"""


class EgoVehicle(Vehicle):
    def __init__(self, world=None):
        image_path = config.INPUT_IMAGE_FP + '/red_car_top_down.png'
        super().__init__(image_path=image_path, world=world)

        self.world = world
        self.ray_endpoints = []
        self.past_positions = []
        self.past_positions_length = 31  # should be an odd number
        self.predictions_fast = []
        self.predictions_slow = []
        self.predicter_slow = Predictions.ConstCurvaturePredicter(self, self.past_positions_length)
        self.predicter_fast = Predictions.ConstCurvaturePredicter(self, 5)
        self.next_wp_id = -1
        self.next_wp = None

        self.future_path_area = None
        self.closest_unknown_points = []
        self.latency = .1  # seconds

        # controlling variable for motion
        self.time_to_col = np.inf
        self.computed_v_long = np.inf
        self.computed_angle = np.inf

        self.computed_acc = None
        self.computed_turn = None

        self.override_acc = None
        self.override_turn = None

        self.overall_desired_acceleration = 0
        self.overall_desired_turn_circle_change = 0

        car = Actor()
        self.ray_length = 100  # in metres
        self.actors = [car]  # pedestrians, cars, bicycles, etc
        self.unknown_areas = None

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
        self.next_wp = self.find_first_waypoint()

    # region drawing functions

    def draw(self, surface):
        self.draw_sensor_rays(surface)
        self.draw_sensor_area(surface)
        self.draw_predicted_motion(surface)
        self.draw_closest_unknown_points(surface)
        self.draw_collision_points(surface)
        self.draw_collision_meta_data(surface)
        self.draw_waypoints(surface)

        super().draw(surface)

    def draw_waypoints(self, surface):
        for p in self.world.waypoints:
            draw_forward_coords = self.convert_world_to_draw_coords(p.position)
            if draw_forward_coords[0]>0:
                pygame.draw.circle(surface, Colours.RED, draw_forward_coords, 10)
        if self.next_wp is not None:
            draw_forward_coords = self.convert_world_to_draw_coords(self.next_wp.position)
            if draw_forward_coords[0]>0:
                pygame.draw.circle(surface, Colours.GREEN, draw_forward_coords, 10)

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
        if unknown_pt_draw_coords[0]>0:
            pygame.draw.circle(surface, Colours.RED, unknown_pt_draw_coords, 10)

        collision_vehicle_draw_coords = self.convert_world_to_draw_coords(self.collision_point_from_vehicle)
        if collision_vehicle_draw_coords[0]>0:
            pygame.draw.circle(surface, Colours.RED, collision_vehicle_draw_coords, 5)
        pygame.draw.aaline(surface, Colours.RED, (self.draw_x, self.draw_y), collision_vehicle_draw_coords)

        collision_actor_draw_coords = self.convert_world_to_draw_coords(self.collision_point_from_actor)
        if collision_actor_draw_coords[0]>0:
            pygame.draw.circle(surface, Colours.BLUE, collision_actor_draw_coords, 5)
        pygame.draw.aaline(surface, Colours.BLUE, unknown_pt_draw_coords, collision_actor_draw_coords)

    def convert_world_to_draw_coords(self, coord):
        x = coord[0] * self.draw_scale + self.draw_offset[0]
        y = coord[1] * self.draw_scale + self.draw_offset[1]
        return x, y

    def draw_closest_unknown_points(self, surface):
        for point in self.closest_unknown_points:
            pt = self.convert_world_to_draw_coords([point[0], point[1]])
            if pt[0]>0:
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
            self.override_acc = self.vel_inc

        if string == "d":
            self.override_acc = -self.vel_inc

        if string == "l":
            self.override_turn = np.sign(self.v_long) * self.turn_inc

        if string == "r":
            self.override_turn = -np.sign(self.v_long) * self.turn_inc

    def update(self, dt):
        super().update(dt)
        self.compute_desired_motion()
        self.collate_information()
        self.accelerate(self.overall_desired_acceleration)
        self.turn_wheel(self.overall_desired_turn_circle_change)

        self.past_positions.append([self.world_x, self.world_y, self.dt])
        if len(self.past_positions) > self.past_positions_length - 1:
            self.past_positions = self.past_positions[-self.past_positions_length:]

    # endregion

    # region class specific functions

    def compute_desired_motion(self):
        # perception
        self.shoot_rays()
        self.calculate_unknown_areas()
        self.calculate_predicted_motion()
        self.calculate_closest_unknown_points()
        self.assess_closest_points()

        # controller
        self.calculate_desired_angle()
        self.calculate_desired_speed()

        # update waypoint if necessary
        self.check_waypoint()

    def collate_information(self):

        self.overall_desired_turn_circle_change = self.computed_angle
        if self.override_turn is not None:
            self.overall_desired_turn_circle_change = self.override_turn
            self.override_turn = None

        if abs(self.overall_desired_turn_circle_change) > 2 * self.turn_inc:
            self.computed_v_long *= 0.5
        self.overall_desired_acceleration = self.computed_v_long - self.v_long

        if self.override_acc is not None:
            self.overall_desired_acceleration = self.override_acc
            self.override_acc = None

    def calculate_desired_speed(self):
        self.computed_v_long = self.max_v

        if np.isfinite(self.time_to_col):
            # if the time to collision is less than the time it would take to brake to 0, plus latency
            # then we need to slow down
            brake_time = abs(self.v_long / self.max_acc_rate)
            if self.time_to_col < self.latency + brake_time:
                self.computed_v_long = self.v_long - np.sign(self.v_long) * self.vel_inc

    def angle_to_point(self, point):
        vector_1 = [(point[0] - self.world_x), (point[1] - self.world_y)]
        vector_2 = [np.cos(self.direction), -np.sin(self.direction)]
        angle = angle_between(vector_1, vector_2)
        # check if we're going clockwise, or counterclockwise

        if not self.is_left(point):
            angle = -angle

        return angle

    def pov_distances(self, point, angle):
        dist = np.sqrt((point[0] - self.world_x) ** 2 + (point[1] - self.world_y) ** 2)
        d_lat = dist * np.sin(angle)
        d_long = dist * np.cos(angle)
        return d_lat, d_long

    def is_left(self, p):
        self.vehicle_forward_point = [
            self.world_x + np.cos(self.direction),
            self.world_y - np.sin(self.direction)]
        return is_left([self.world_x, self.world_y], self.vehicle_forward_point, p)

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

        for p in self.closest_unknown_points:
            # find the angle, lateral and longitudinal distances for this point
            # d_lat and angle are positive if the obstacle point is to the left of the vehicle, negative to the right
            # this is consistent if driving backwards (note that the forward direction is maintained in this case)

            angle = self.angle_to_point(p)
            d_lat, d_long = self.pov_distances(p, angle)

            if d_long == 0:
                continue

            # this is used for ensuring we get the right quadrants
            obs_sign = np.sign(angle)
            vel_sign = np.sign(self.v_long)

            for actor in self.actors:
                # there are up to two angles. see powerpoint for derivation
                # we take the minimum angle, correct for cw/ccw and direction of vehicle speed
                # then make it relative to the world axis
                a_1, a_2 = self.find_worst_case_rel_angle(d_lat, d_long, actor.speed)
                if not np.isfinite(a_1) and not np.isfinite(a_2):
                    continue

                rel_worst_case_angle = min(a_1, a_2) * obs_sign * vel_sign
                world_worst_case_angle = (rel_worst_case_angle + np.pi + self.direction) \
                                         % (2 * np.pi)  # to make it world direction, and not relative

                # this bit is kinematics and the cosine rule
                rel_dist = np.sqrt((p[0] - self.world_x) ** 2 + (p[1] - self.world_y) ** 2)
                rel_vel = np.sqrt(actor.speed ** 2 + self.v_long ** 2 - \
                                  (2 * actor.speed * vel_sign * self.v_long * np.cos(np.pi - rel_worst_case_angle)))

                time_to_col = rel_dist / rel_vel

                if time_to_col < self.time_to_col:
                    self.time_to_col = time_to_col
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

                if abs(angle) > np.pi / 2 and np.sign(self.v_long) > 0:
                    # pygame.draw.circle(surface, Colours.RED, (point.x, point.y), 5)
                    continue
                if abs(angle) < np.pi / 2 and np.sign(self.v_long) < 0:
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

    # region Controller
    def calculate_desired_angle(self):
        if self.next_wp is None:
            self.next_wp = self.find_first_waypoint()
            self.next_wp_id = self.next_wp.guid
            return

        self.computed_angle = self.angle_to_point(self.next_wp.position)

    def check_waypoint(self):
        if self.next_wp is None:
            self.next_wp = self.find_first_waypoint()
            self.next_wp_id = self.next_wp.guid
            return

        dist_sqr = (self.world_x - self.next_wp.position[0]) ** 2 + (self.world_y - self.next_wp.position[1]) ** 2
        if dist_sqr < 10:
            self.next_wp = self.find_next_waypoint()
            if self.next_wp is None:
                self.next_wp = self.find_first_waypoint()
            self.next_wp_id = self.next_wp.guid

    def find_first_waypoint(self):
        nearest_point_id = np.inf
        next_wp_temp = None
        for p in self.world.waypoints:
            if p.guid < nearest_point_id:
                nearest_point_id = p.guid
                next_wp_temp = p
        return next_wp_temp

    def find_next_waypoint(self):
        nearest_point_id = np.inf
        next_wp_temp = None
        for p in self.world.waypoints:
            if self.next_wp_id < p.guid < nearest_point_id:
                nearest_point_id = p.guid
                next_wp_temp = p

        return next_wp_temp

    # endregion
