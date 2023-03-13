from abc import ABC
from shapely.geometry import LineString
from src.Model.LocomotionUtilityClasses.Waypoint import Waypoint
import numpy as np

class Optimiser(ABC):
    def __init__(self, clock):
        ''' creates the optimiser'''
        self.clock=clock

class BasicOptimiser(Optimiser):
    def __init__(self, agent):
        ''' creates the optimiser'''
        super().__init__(agent.clock)
        self.agent=agent

    def plan_path(self):
        '''
            creates and returns an straight line path
        '''
        continuous_route = self._plan_linear_continuous_route()
        return continuous_route

    def plan_trajectory(self):
        '''
            should consider things like acceleration, travel time, perceived discomfort, max speed, etc.
            at the moment it just draws straight lines

            you should pass in a single cognitive state into the trajectory planner.
            then the optimiser should generate trajectories
            then the cost of those trajectories should be calculated, against cognitive state
        :param continuous_path:
        :return: tickstep_route
        '''
        straight_line_path = self.plan_path()
        continuous_path=straight_line_path
        start_tick=self.agent.starting_tick
        tickstep_route = self._convert_continuous_to_waypoints(continuous_path, start_tick)
        return tickstep_route

    def _convert_continuous_to_waypoints(self, continuous_route, start_tick=0):
        """
        :param continuous_route: must be made of shapely linestrings
        :return: a tickstep calculated route. this should be changed to a tickstep route of waypoints?
        """
        speed = self.agent.characteristics['speed']# maybe that's actually an optimisation outcome?
        tickstep_route = {}
        lines = []
        line_start_tick = start_tick

        for i in range(len(continuous_route.xy[0]) - 1):
            line = LineString([(continuous_route.xy[0][i], continuous_route.xy[1][i]),
                               (continuous_route.xy[0][i + 1], continuous_route.xy[1][i + 1])])
            lines.append(line)

        for i in lines:
            dist = i.length
            time = dist / speed
            tick_steps = int(time * self.clock.ticks_per_second / self.clock.tickstep)
            angle = np.arctan2(i.coords[-1][1] - i.coords[0][1], i.coords[-1][0] - i.coords[0][0])
            angle= angle if angle >= 0 else angle + 2*np.pi

            for j in range(tick_steps):
                interpolation_value = j / tick_steps * dist
                interp_point = i.interpolate(interpolation_value)
                tick_value = j*self.clock.tickstep + line_start_tick
                temp_waypoint = Waypoint(interp_point.xy[0][0], interp_point.xy[1][0],tick_value, speed, speed, angle )
                tickstep_route[tick_value]= temp_waypoint
            line_start_tick = max(tickstep_route.keys())

        return tickstep_route

    def _plan_linear_continuous_route(self):
        """ create straight lines between each of the points in the route """
        points=[]
        points.append((self.agent.origin[0], self.agent.origin[1]))
        for i in self.agent.route:

            points.append((self.agent.route[i][0], self.agent.route[i][1]))
        continuous_route=LineString(points)

        return continuous_route

    def replan_local_trajectory(self):
        '''
        here we can replan the local trajectory to accommodate local issues
        :return:

        waypoints are defined using the following variables.
        x, y,
        vx, vy
        they are also defined using a fixed dt.

        here is what a future waypoint looks like, based on current waypoints.
        x(t+dt)=x(t)+dt*vx
        y(t+dt)=y(t)+dt*vy

        these are the predicted variables that can be used to optimise a trajectory (an ordered set of waypoints)
        X3, Y3, VX3, VY3
        X2, Y2, VX2, VY2, etc.

        the constraints on the optimisation are:

        minimise time to destination:
            sum_t of waypoints.t from now to destination, this will minimise number of points to destination

        constrain acceleration
            this limits the change in velocity from waypoint to waypoint. negative acc is easier than positive

            minimise longitudinal acceleration: norm_2 (v_l (t+1) - v_l(t) )
            minimise transverse acc: norm_2 (v_t(t+1) - v_t(t))

        # interactions between people- this is the difficult element of the optimisation.
        a minimum distance: norm_2 (xi-self.x, yi-self.y)>= r_comfort


        i think do the bubble stretching algorithm. adapt it for moving obstacles.
         A Convex Optimization Approach to Smooth Trajectories for Motion Planning with Car-Like Robots
        Zhijie Zhu Edward Schmerling Marco Pavone

        @inproceedings{inproceedings,
        author = {Zhu, Zhijie and Schmerling, Edward and Pavone, Marco},
        year = {2015},
        month = {12},
        pages = {835-842},
        title = {A convex optimization approach to smooth trajectories for motion planning with car-like robots},
        doi = {10.1109/CDC.2015.7402333}
        }

        this will avoid obstacles (such as predicted position of neighbours, and walls), while optimising
        the path for speed.
       '''





        five_seconds_of_ticks= int(5 * self.clock.ticks_per_second / self.clock.tickstep)
        for i in range(five_seconds_of_ticks):
            t= self.agent.global_ticks + self.tickstep * i
            if t not in self.agent.current_own_trajectory.keys():
                return self.agent.current_own_trajectory
            addition=0#self.rng.integers(-1, 1, 1)[0]*0 * np.cos(self.internal_ticks * 0.002)
            previous_wp= self.agent.current_own_trajectory[t]
            self.agent.current_own_trajectory[t]=Waypoint.Waypoint(
                previous_wp.position[0]+addition,
                previous_wp.position[1]+addition,
                t,
                previous_wp.velocity[0],
                previous_wp.velocity[1],
                previous_wp.direction
            )

