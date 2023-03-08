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

    # region drawing functions

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

            if angle > np.pi / 2:
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

    # endregion

    # region class specific functions
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
        self.collision_points_from_actor = []
        self.collision_points_from_vehicle = []
        self.is_limiting_speed = False
        self.time_to_col = np.inf
        vehicle_forward_point = [
            self.world_x + 10 * np.sign(self.v_long) * self.v_long * np.cos(self.direction),
            self.world_y - 10 * np.sign(self.v_long) * self.v_long * np.sin(self.direction)]

        for p in self.closest_unknown_points:
            # find the angle, lateral and longitudinal distances for this point
            # d_lat and angle are positive if the obstacle point is to the left of the vehicle, negative to the right
            # this is consistent if driving backwards (note that the forward direction is maintained in this case)

            angle = self.angle_to_point(p)
            if not is_left([self.world_x, self.world_y], vehicle_forward_point, p):
                angle = angle - np.pi / 2
            d_lat, d_long = self.pov_distances(p, angle)
            obs_sign = np.sign(angle)

            for actor in self.actors:
                # at the moment it only works when driving forward, with the point on the left hand side
                # TODO: i'm fairly certain the issue is here.
                #  the min value is correct- maybe the find_worst_case_rel_angle function is wrong?
                #  if not, it's what we do with that rel_worst_case_angle to turn it into world worst case
                a_1, a_2 = self.find_worst_case_rel_angle(d_lat, d_long, actor.speed)
                rel_worst_case_angle = min(a_1, a_2) * obs_sign
                world_worst_case_angle = (rel_worst_case_angle + np.pi + self.direction) \
                                         % (2 * np.pi)  # to make it world direction, and not relative

                rel_dist = np.sqrt((p[0] - self.world_x) ** 2 + (p[1] - self.world_y) ** 2)
                rel_vel = np.sqrt(
                    actor.speed ** 2 +
                    self.v_long ** 2 -
                    (2 * actor.speed * abs(self.v_long) * np.cos(np.pi - abs(rel_worst_case_angle))))

                time_to_col = rel_dist / rel_vel

                vehicle_collision_point = [
                    self.world_x + time_to_col * self.v_long * np.cos(self.direction),
                    self.world_y - time_to_col * self.v_long * np.sin(self.direction)]

                actor_collision_point = [
                    p[0] + time_to_col * actor.speed * np.cos(world_worst_case_angle),
                    p[1] - time_to_col * actor.speed * np.sin(world_worst_case_angle)]

                self.collision_points_from_vehicle.append(vehicle_collision_point)
                self.collision_points_from_actor.append(actor_collision_point)

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
                if dist < closest_dist:
                    self.closest_unknown_points = [pt]
                    closest_dist = dist

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
        number_rays = 500
        for i in range(number_rays):
            # create a line between the car and the max length
            # cycle through all obstacles
            target_x = self.world_x + np.cos(angle) * depth
            target_y = self.world_y - np.sin(angle) * depth
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
        x = abs(d_lat) / d_long
        y = abs(self.v_long) / v_actor

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
