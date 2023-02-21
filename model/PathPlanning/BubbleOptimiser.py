import copy

from model.LocomotionUtilityClasses.Waypoint import Waypoint
from model.PathPlanning.OptimisationClasses import Optimiser

from dataclasses import dataclass, field
from shapely.geometry import LineString, Point
import cvxpy as cp
from typing import List
import numpy as np


@dataclass
class Bubble:
    radius: float = 0
    ticks: float = 0
    direction: float = 0
    centre: List[float] = field(default_factory=List)
    acceleration = 0
    speed = 0

def point_point_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return np.sqrt(dx ** 2 + dy ** 2)

def find_average_length(bubbles):
    d = 0
    for i in range(1, len(bubbles) - 1):
        d += point_point_distance(bubbles[i].centre, bubbles[i - 1].centre)
    d /= (len(bubbles) - 1)
    return d

def get_unit_normal_from_obstacle(point, obstacle):
    """
    get the direction away from the centroid of the polygon
    """
    centre = obstacle.centroid.coords[0]
    grad = [point[0] - centre[0], point[1] - centre[1]]
    return grad / np.linalg.norm(grad)

class BubbleOptimiser(Optimiser):
    def __init__(self, agent):
        super().__init__(agent.clock)
        self.agent = agent
        self.r_u, self.r_l = 10, .5
        self.fixed_obstacles = []
        self.dynamic_obstacles = []
        self.bubbles = []

    def plan_trajectory(self, overwrite=False):
        '''
            should consider things like acceleration, travel time, perceived discomfort, max speed, etc.
            at the moment it just draws straight lines

            you should pass in a single cognitive state into the trajectory planner.
            then the optimiser should generate trajectories
            then the cost of those trajectories should be calculated, against cognitive state
        :param continuous_path:
        :return: tickstep_route
        '''
        if not overwrite:
            self.fixed_obstacles=self.agent.environment.fixed_obstacles
            self.continuous_path = self.create_continuous_path()
            self.waypoints = self._convert_continuous_to_waypoints(self.continuous_path)

        self.bubbles = self.create_bubbles()
        optimised_positions = self.optimise_force()
        if optimised_positions is not None:
            sparse_waypoints = self.create_waypoints_from_optimised_positions(optimised_positions)
            max_key=max(list(self.waypoints.keys()))
            self.waypoints= self.interpolate_waypoints(sparse_waypoints, max_key)

        return self.waypoints

    def overwrite_trajectory(self, obstacles, trajectory):
        self.waypoints = trajectory
        self.dynamic_obstacles = obstacles
        return self.plan_trajectory(True)

    def create_continuous_path(self):
        """
            creates and returns an straight line path
        """
        points = [(self.agent.origin[0], self.agent.origin[1])]
        for i in self.agent.route:
            points.append((self.agent.route[i][0], self.agent.route[i][1]))
        continuous_route = LineString(points)
        return continuous_route

    def interpolate_waypoints(self, sparse_waypoints, max_key):
        tickstep_route = {}
        tickstep = self.agent.clock.tickstep

        start_tick = list(sparse_waypoints.keys())[0]
        end_tick = list(sparse_waypoints.keys())[-1]
        num_ticksteps = int((end_tick-start_tick)/tickstep)

        current_sparse_wp_idx = 1
        prev_wp = sparse_waypoints[start_tick]
        next_wp = sparse_waypoints[list(sparse_waypoints.keys())[current_sparse_wp_idx]]
        tickstep_route[start_tick] = prev_wp

        for i in range(1, num_ticksteps):
            tick = start_tick+i*tickstep
            if tick in sparse_waypoints.keys():
                temp_waypoint = sparse_waypoints[tick]
                current_sparse_wp_idx += 1
                if current_sparse_wp_idx < len(sparse_waypoints.keys()):
                    prev_wp = next_wp
                    next_wp = sparse_waypoints[list(sparse_waypoints.keys())[current_sparse_wp_idx]]

            else:
                interp_value = (tick-prev_wp.t) / (next_wp.t-prev_wp.t)
                x=interp_value*(next_wp.position[0]-prev_wp.position[0]) +prev_wp.position[0]
                y=interp_value*(next_wp.position[1]-prev_wp.position[1]) +prev_wp.position[1]
                speed_x=interp_value*(next_wp.velocity[0]-prev_wp.velocity[0]) +prev_wp.velocity[0]
                speed_y=interp_value*(next_wp.velocity[1]-prev_wp.velocity[1]) +prev_wp.velocity[1]
                angle=interp_value*(next_wp.direction-prev_wp.direction) +prev_wp.direction
                temp_waypoint = Waypoint(x,y, tick, speed_x, speed_y, angle)

            tickstep_route[tick] = temp_waypoint

        #tickstep_route[start_tick+num_ticksteps*tickstep]=Waypoint(x,y, tick, speed_x, speed_y, angle)
        return tickstep_route

    def create_waypoints_from_optimised_positions(self, optimised_positions):
        tickstep_route = {}

        for i in range(len(optimised_positions[0])):
            x = optimised_positions[0][i]
            y = optimised_positions[1][i]
            tick = self.bubbles[i].ticks
            angle = self.bubbles[i].direction
            speed = self.bubbles[i].speed
            temp_waypoint = Waypoint(x, y, tick, speed * np.cos(angle), speed * np.sin(angle), angle)
            tickstep_route[tick] = temp_waypoint
        return tickstep_route

    def _convert_continuous_to_waypoints(self, continuous_route, start_tick=0):
        """
        :param continuous_route: must be made of shapely linestrings
        :return: a tickstep calculated route. this should be changed to a tickstep route of waypoints?
        """
        speed = self.agent.characteristics['speed']  # maybe that's actually an optimisation outcome?
        tickstep_route = {}
        lines = []
        line_start_tick = 0#self.agent.starting_tick

        for i in range(len(continuous_route.xy[0]) - 1):
            line = LineString([(continuous_route.xy[0][i], continuous_route.xy[1][i]),
                               (continuous_route.xy[0][i + 1], continuous_route.xy[1][i + 1])])
            lines.append(line)

        for i in lines:
            dist = i.length
            time = dist / speed
            tick_steps = int(time / self.clock.tickstep * self.clock.ticks_per_second)
            angle = np.arctan2(i.coords[-1][1] - i.coords[0][1], i.coords[-1][0] - i.coords[0][0])
            angle = angle if angle >= 0 else angle + 2 * np.pi

            # need to make sure the first point isn't within an obstacle, becos that'll fuck shit up
            # check if first point is in obstacle, if so, create waypoint, translate using Translate bubble,
            # otherwise just create waypoint
            #self.CheckFirstWaypointNotInObstacle(line, tickstep_route, tick_value, speed, angle)

            for j in range(tick_steps):
                interpolation_value = j / tick_steps * dist
                interp_point = i.interpolate(interpolation_value)
                tick_value = j * self.clock.tickstep + line_start_tick
                temp_waypoint = Waypoint(interp_point.xy[0][0], interp_point.xy[1][0], tick_value, speed, speed, angle)
                tickstep_route[tick_value] = temp_waypoint
            line_start_tick = tick_value# max(tickstep_route.keys())

        return tickstep_route

    def CheckFirstWaypointNotInObstacle(self, line, tickstep_route, tick_value, speed, angle):

        start_point = [line.xy[0][0], line.xy[1][0]]
        point=start_point
        radius, nearest_obstacle = self.find_least_radius_from_obstacles(start_point)
        if radius == 0:
            wp=Waypoint(start_point[0], start_point[1], tick_value, speed, speed, angle)
            radius, point = self.translate_bubble(radius, wp, nearest_obstacle)

        temp_waypoint = Waypoint(point[0], point[1], tick_value, speed, speed, angle)
        tickstep_route[tick_value] = temp_waypoint
        return temp_waypoint

    def find_least_radius_from_obstacles(self, point):
        radius = 9999999
        nearest_obstacle = []
        tp = Point(point)

        for i in self.fixed_obstacles:
            if i.contains(tp):
                # move the waypoint away from the centre of the obstacle?
                # the main route should never collide with an obstacle, but the dynamic route might (esp with predictions).
                return 0, i
            dist_from_edge = i.distance(tp)
            if dist_from_edge < radius:
                radius = dist_from_edge
                nearest_obstacle = i

        for i in self.dynamic_obstacles:
            if i.contains(tp):
                return 0, i
            dist_from_edge = i.distance(tp)
            if dist_from_edge < radius:
                radius = dist_from_edge
                nearest_obstacle = i
        return radius, nearest_obstacle

    def generate_bubble(self, waypoint):
        # If the obstacles have more general polygonal shapes, a bisection search is
        # performed with respect to the bubble radius.
        point = waypoint.position
        radius, nearest_obstacle = self.find_least_radius_from_obstacles(point)
        tickstamp = waypoint.t
        if radius < self.r_l:
            radius, point = self.translate_bubble(radius, waypoint, nearest_obstacle)
        radius = min(radius, self.r_u)
        direction = waypoint.direction
        return Bubble(radius, tickstamp, direction, point)

    def translate_bubble(self, radius, waypoint, nearest_obstacle):
        initial_radius = radius
        point = waypoint.position
        direction = get_unit_normal_from_obstacle(point, nearest_obstacle)
        while radius < self.r_l:
            point = [point[0] + direction[0] * .1, point[1] + direction[1] * .1]
            radius, nearest_obstacle = self.find_least_radius_from_obstacles(point)



            if radius < initial_radius:
                # can occur with weird, non-convex polygons- reverse the direction, add some randomness, and hope that helps.
                direction = [-direction[1] + 0.1 * np.random.rand(), direction[0] + 0.1 * np.random.rand()]
                initial_radius = radius
        return radius, point

    def optimise_force(self):
        bubbles = self.bubbles
        n = len(bubbles)
        x = cp.Variable((2, n))

        objective = 0
        d = find_average_length(bubbles)

        constr = [x[:, 0] == (bubbles[0].centre[0], bubbles[0].centre[1]),
                  x[:, 1] == (bubbles[1].centre[0], bubbles[1].centre[1]),
                  x[:, n - 2] == (bubbles[n - 2].centre[0], bubbles[n - 2].centre[1]),
                  x[:, n - 1] == (bubbles[n - 1].centre[0], bubbles[n - 1].centre[1])]
        for k in range(2, n - 2):
            objective += cp.sum_squares(2 * x[:, k] - x[:, k - 1] - x[:, k + 1])
            # need to add one final constraint- alpha_k d**2/v_k. need to add a speed and acceleration for each bubble.
            constr.append(cp.norm(2 * x[:, k] - x[:, k - 1] - x[:, k + 1])
                          <= min(d ** 2 / 1, bubbles[k].acceleration * (d / (bubbles[k].speed+1e-10)) ** 2))

            constr.append(cp.norm(x[:, k] - bubbles[k].centre) <= bubbles[k].radius)
        problem = cp.Problem(cp.Minimize(objective),
                             constr)  # The optimal objective value is returned by `prob.solve()`.
        problem.solve()
        return x.value

    def create_bubbles(self):
        first_key = min(list(self.waypoints.keys()))
        bubbles = [self.generate_bubble(self.waypoints[first_key])]
        bubbles[0].speed = np.size(self.waypoints[first_key].velocity)

        for key, value in self.waypoints.items():
            cutoff = .5 * bubbles[-1].radius
            d = point_point_distance(value.position, bubbles[-1].centre)
            if d < cutoff:
                continue
            else:
                b = self.generate_bubble(value)  # generate bubble automatically calls translate bubble
                d = point_point_distance(b.centre, bubbles[- 1].centre)
                if d < cutoff:
                    continue
                bubbles.append(b)


        for i in range(1, len(bubbles) - 1):
            ds_pre = point_point_distance(bubbles[i].centre, bubbles[i - 1].centre)
            dt_pre = bubbles[i].ticks - bubbles[i - 1].ticks

            ds_post = point_point_distance(bubbles[i].centre, bubbles[i + 1].centre)
            dt_post = bubbles[i].ticks - bubbles[i + 1].ticks

            bubbles[i].speed = 0.5 * (ds_pre / dt_pre + ds_post / dt_post)
            dtheta = bubbles[i].direction - bubbles[i - 1].direction

            dv = bubbles[i].speed - bubbles[i - 1].speed
            dv_lat = dv * np.sin(dtheta)
            dv_long = dv * np.cos(dtheta)
            bubbles[i].acceleration = (np.sqrt(dv_long ** 2 + dv_lat ** 2)) / dt_pre

        return bubbles
