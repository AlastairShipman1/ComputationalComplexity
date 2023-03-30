import shapely
from shapely.ops import unary_union
from shapely.geometry import LineString, Point
from model import clock
from model.Agent import Agent
from model.LocomotionUtilityClasses import Waypoint
import numpy as np
from simple_pid import PID


def rotate_point(centre, point, radians):

    rel_x = point[0] - centre[0]
    rel_y = point[1] - centre[1]

    agent_circle_rotated_pos = [rel_x * np.cos(radians) - rel_y * np.sin(radians),
                                rel_x * np.sin(radians) + rel_y * np.cos(radians)]
    post_rotated_pos = [agent_circle_rotated_pos[0] + centre[0],
                        agent_circle_rotated_pos[1] + centre[1]]
    return post_rotated_pos


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


class Human(Agent.Agent):
    # region Initial Functions
    def __init__(self, clock: clock, uuid):
        """subscribe to the clock tick, and initialise the required variables"""
        super().__init__(clock, uuid)
        self.initialise_variables()
        self.clock.tick_events.on_change += self.tick
        self.clock.post_tick_events.on_change += self.post_tick
        self.clock.pre_tick_events.on_change += self.pre_tick

    def initialise_variables(self):
        """initialise time, updating limits, characteristics, predictions and position tracking"""
        # optimise the path from current position to destination within that field to create a trajectory.
        self.simulating = False
        self.completed = False

        self.internal_ticks = 0
        self.global_ticks = self.clock.ticks
        self.tickstep = self.clock.tickstep
        self.regular_update_limit = 350  # ticks
        self.regular_updater_tick_tracker = self.regular_update_limit  # make sure you perform predictions at first timestep
        self._starting_tick = 0

        self.rng = self.clock.rng
        self.current_destination_index = 0
        self.neighbours = None

        self.desired_speed = 1 + self.clock.rng.uniform(-.1, .1)
        self.fov = np.deg2rad(45)
        self.characteristics = {'speed': self.desired_speed, 'fov': self.fov}  # fov should be a function of speed?

        self.visible_neighbours = {}
        self.current_neighbour_predictions = {}
        self.all_predictions = {}
        self.current_own_trajectory = {}
        self.past_trajectory = {}
        self.predicted_collisions = {}
        self.proposed_next_position = []
        self.next_waypoint = None
        self._position = [0, 0]
        self._velocity = [0, 0]
        self._direction = np.pi/2

        self.sigma_ttc = 1.8 #1.8
        self.sigma_dca = 0.1 #0.1
        self.sigma_speed = 3.3 #3.3
        self.sigma_bearing = 2 #2

        self.pid_x = PID(1, .001, 0.0005, sample_time=None)
        self.pid_y = PID(1, .001, 0.0005, sample_time=None)

        arm1 = shapely.geometry.Point(0, 0.4).buffer(.25)
        arm2 = shapely.geometry.Point(0, -0.4).buffer(.25)
        body = shapely.geometry.Point(0, 0).buffer(.35)
        self._shape = unary_union([arm1, arm2, body])
        # self._shape=shapely.geometry.Polygon([(1,1), (0,1), (0,0)])

        # self.neighbour_prediction_model = PredictionClasses.LinearPredictions(self.clock)
        # self.trajectory_optimisation_model = OptimisationClasses.BasicOptimiser(self) #BubbleOptimiser.BubbleOptimiser(self)

    def set_origin(self, origin):
        self.origin = origin
        self._position = self.origin

    def set_route(self, route):
        self.route = route
        self.set_destination(self.route[0])

    # endregion

    # region Clocktick Functions

    def pre_tick(self):
        """ before clock ticks, this is called"""
        if self.completed or (self.clock.ticks + self.clock.tickstep) < self.starting_tick:
            return
        if not self.simulating:
            self.begin_simulating()

        if self.regular_updater_tick_tracker >= self.regular_update_limit:
            self.regular_updater_tick_tracker = 0
            # self.make_predictions_for_neighbours()
            # self.store_predictions_for_visualisation()

    def tick(self):
        """ every time the clock ticks, this is called."""
        if self.completed or (self.clock.ticks + self.clock.tickstep) < self.starting_tick:
            return

        self.plan_next_position()
        self.move()
        self.store_position_for_visualisation()

        self.at_destination = self.check_at_destination()
        if self.at_destination:

            if self.current_destination_index < max(self.route.keys()):
                self.update_destination()
                self.at_destination = False
            else:
                self.completed = True

        self.update_internal_clock()
        # self.set_next_waypoint()

    def post_tick(self):
        """ after every clock tick, this is called"""
        if self.completed or (self.clock.ticks + self.clock.tickstep) < self.starting_tick:
            return
        # self.predicted_collisions = self.detect_collisions()

        if len(self.predicted_collisions.keys()) > 0:
            self.generate_locally_optimised_trajectory()

    def begin_simulating(self):
        """
        this is called when the global ticks are first larger than the _starting_tick
        :return:
        """
        self.simulating = True
        self.global_ticks = self.clock.ticks
        # self.generate_base_trajectory()
        # self.next_waypoint = self.current_own_trajectory[self.starting_tick]

    # endregion

    # region Properties

    @property
    def starting_tick(self):
        return self._starting_tick

    @starting_tick.setter
    def starting_tick(self, start_tick):
        if not (isinstance(start_tick, int) and start_tick >= 0):
            raise ValueError("Tick needs to be an integer, bigger than or equal to zero")
        self._starting_tick = start_tick

    @property
    def position(self):
        return self._position

    @property
    def velocity(self):
        return self._velocity

    @property
    def direction(self):
        return self._direction

    @property
    def environment(self):
        return self._environment

    @environment.setter
    def environment(self, env):
        """sets environment"""
        self._environment = env

    @property
    def unique_id(self):
        """ gets the id of the agent"""
        return self._unique_id

    @property
    def clock(self):
        """ gets the clock of the agent"""
        return self._clock

    @property
    def shape(self):
        """ defines the shape of the agent"""
        return self._shape

    # endregion

    # region Utils

    def set_destination(self, destination):
        self.destination = destination

    def update_destination(self):

        self.current_destination_index += 1
        self.set_destination(self.route[self.current_destination_index])

    def get_neighbours(self):
        self.neighbours = self.environment.agents.copy()
        if self in self.neighbours:
            self.neighbours.remove(self)

    def get_visible_neighbours(self):
        self.neighbours = self.environment.agents.copy()
        if self in self.neighbours:
            self.neighbours.remove(self)
        for neighbour in self.neighbours[:]:
            if not self.can_see_neighbour(neighbour):
                self.neighbours.remove(neighbour)

    def can_see_neighbour(self, neighbour):
        vector_1 = [neighbour.position[0] - self._position[0], neighbour.position[1] - self._position[1]]
        vector_2 = [np.cos(self._direction), np.sin(self._direction)]
        angle = angle_between(vector_1, vector_2)
        return angle < self.characteristics['fov']

    def store_position_for_visualisation(self):
        """ store position as a waypoint for visualisation"""
        wp = Waypoint.Waypoint(self._position[0], self._position[1], self.clock.ticks, self._velocity[0],
                               self._velocity[1], self._direction)
        self.past_trajectory[self.clock.ticks] = wp

    def store_predictions_for_visualisation(self):
        self.all_predictions[self.clock.ticks] = self.current_neighbour_predictions

    def update_internal_clock(self):
        self.internal_ticks += self.tickstep
        self.regular_updater_tick_tracker += self.tickstep
        self.global_ticks = self.clock.ticks

    def set_next_waypoint(self):
        self.find_next_waypoint()
        return

    def find_next_waypoint(self):
        # TODO: implement this
        ...

    def check_at_destination(self):

        distance = (self._position[0] - self.destination[0]) ** 2 \
                   + (self._position[1] - self.destination[1]) ** 2
        return distance < .1

    # endregion

    # region Model Functions
    def detect_collisions(self):
        """
         go through all predictions, check whether waypoints intersect the self._shape (do this by moving the points, not
         the shape, for efficiency).
        :return: bool if trajectory needs to change
        """
        predicted_collisions = {}
        for i in self.current_neighbour_predictions:
            # these are the timestamps
            tick = self.global_ticks + self.tickstep * i
            if tick not in self.current_own_trajectory.keys():
                continue

            predicted_collisions[tick] = []
            own_predicted_pos = self.current_own_trajectory[tick].position

            for agent in self.current_neighbour_predictions[i]:
                agent_pos = self.current_neighbour_predictions[i][agent].position
                x_delta = own_predicted_pos[0] - agent_pos[0]
                y_delta = own_predicted_pos[1] - agent_pos[1]

                if x_delta ** 2 + y_delta ** 2 < 1:
                    predicted_collisions[tick].append(agent_pos)

            if len(predicted_collisions[tick]) == 0:
                del predicted_collisions[tick]

        return predicted_collisions

    def plan_next_position(self):
        """
        Plans next position, stores it in self.proposed_next_position
        """

        # Previous, waypoint-based approach
        dt = self.clock.tickstep / self.clock.ticks_per_second
        dx=self.next_waypoint.position[0]-self.position[0]
        dy=self.next_waypoint.position[1]-self.position[1]
        update_x=dx
        update_y=dy

        deviation = self.next_waypoint.assess_deviation(self)
        update_x = self.pid_x(deviation["position"][0], dt) \
            +self.rng.integers(-1, 2, 1)[0] * np.cos(self.internal_ticks * 0.002)* .1 * dt
        update_y = self.pid_y(deviation["position"][1], dt) \
            +self.rng.integers(-1, 2, 1)[0] * np.sin(self.internal_ticks * 0.002)* .1 * dt
        self.proposed_next_position = [self._position[0] + update_x, self._position[1] + update_y]
        '''
        # Gradient based approach

        direction_to_goal = np.arctan2(self.destination[1] - self.position[1], self.destination[0] - self.position[0])
        delta_direction = direction_to_goal - self.direction

        current_speed = np.linalg.norm(self.velocity)
        delta_speed = current_speed - self.desired_speed

        self.get_visible_neighbours()
        # self.neighbours=[]
        dco_da = 0
        dco_ds = 0
        n_count = 0
        for object in self.neighbours:
            dc_ds_i, dc_da_i = self.get_obstacle_differential_costs(object)
            if np.isfinite(dc_ds_i) and np.isfinite(dc_da_i):
                dco_ds += dc_ds_i
                dco_da += dc_da_i
                n_count += 1

        dco_da /= max(1, n_count)
        dco_ds /= max(1, n_count)

        dcm_da = np.e ** (-0.5 * (delta_direction / self.sigma_bearing) ** 2) *(-delta_direction) / (2 * self.sigma_bearing ** 2)
        dcm_ds = np.e ** (-0.5 * (delta_speed / self.sigma_speed) ** 2) * delta_speed / (2 * self.sigma_speed ** 2)

        del_Ct = [dcm_ds + dco_ds, dcm_da + dco_da]

        if current_speed == 0:
            current_speed = self.clock.rng.uniform(0, 1)

        direction = self.direction
        lambda_n = 0.7

        new_direction = direction - lambda_n * del_Ct[1] * direction + self.clock.rng.uniform(-0.1, 0.1)
        new_speed = current_speed - lambda_n * del_Ct[0] * current_speed + self.clock.rng.uniform(-0.1, 0.1)

        dt = self.clock.tickstep / self.clock.ticks_per_second
        dx = new_speed * np.cos(new_direction) * dt
        dy = new_speed * np.sin(new_direction) * dt
        self.proposed_next_position = [self.position[0] + dx, self.position[1] + dy]
        '''

    def get_closest_approach_metrics(self, rel_position, rel_velocity):
        """
        given a relative position [x, y], and a relative velocity [v_x, v_y]
        we need to return the time and distance of closest approach
        given we have relative values, and we assume constant velocity
        we just need to:
            check if the distance is getting bigger or smaller
            (i.e. the abs(angle) between rel_pos and rel_vel is more than 90 degrees)
            -- or could just include negative values?

            and then take the abs(rel_pos)*sin(rel_angle) for closest distance,
            and the abs(rel_pos)*cos(rel_angle)/abs(rel_vel) for the time for this to occur
            (it's easier to draw this out if confused)


        rel_angle = angle_between(rel_position, rel_velocity)
        if abs(rel_angle) > np.pi:
            return np.inf, np.inf

        mag_p = np.linalg.norm(rel_position)

        distance = mag_p * np.sin(rel_angle)
        time = mag_p * np.cos(rel_angle) / np.linalg.norm(rel_velocity)
        """

        # the paper does it this way, so lets do it this way
        if rel_velocity == [0, 0]:
            return np.inf, rel_position[0], rel_position[1]

        time = -np.dot(rel_position, rel_velocity) / (rel_velocity[0] ** 2 + rel_velocity[1] ** 2)
        rel_x = rel_position[0] + time * rel_velocity[0]
        rel_y = rel_position[1] + time * rel_velocity[1]

        return time, rel_x, rel_y


    def get_obstacle_differential_costs(self, obstacle):
        p, v = obstacle.position, obstacle.velocity
        p_r = [p[0] - self.position[0], p[1] - self.position[1]]
        v_r = [v[0] - self.velocity[0], v[1] - self.velocity[1]]

        ttca, dca_x, dca_y = self.get_closest_approach_metrics(p_r, v_r)
        dca = np.sqrt(dca_x ** 2 + dca_y ** 2)

        dvr_ds = [np.cos(self.direction), np.sin(self.direction)]
        dvr_da = [self.velocity[1], -self.velocity[0]]

        dttc_ds = np.dot(np.add(p_r, np.dot(2 * ttca, v_r)), dvr_ds) / (np.dot(v_r, v_r))
        dttc_da = -np.dot(np.add(p_r, np.dot(2 * ttca, v_r)), dvr_da) / (np.dot(v_r, v_r))

        ddca_ds = np.dot([dca_x, dca_y], (np.add(np.dot(dttc_ds, v_r), np.dot(-ttca, dvr_ds)))) / dca
        ddca_da = np.dot([dca_x, dca_y], (np.add(np.dot(dttc_da, v_r), np.dot(ttca, dvr_da)))) / dca

        c_i = np.e ** (-0.5 * ((ttca / self.sigma_ttc) ** 2 + (dca / self.sigma_dca) ** 2))

        dco_ds = -c_i * ((dttc_ds * ttca / (self.sigma_ttc ** 2)) - (ddca_ds * dca / (self.sigma_dca ** 2)))
        dco_da = -c_i * ((dttc_da * ttca / (self.sigma_ttc ** 2)) - (ddca_da * dca / (self.sigma_dca ** 2)))

        return dco_ds, dco_da


    def get_base_direction(self):
        """
        always tries to get to the next waypoint (which is defined by the one closest in future time)
        :return:
        """
        next_tick = self.global_ticks + self.clock.tickstep

        if next_tick > max(self.current_own_trajectory.keys()):
            self.completed = True
            return 0

        next_waypoint = self.current_own_trajectory[next_tick]
        direction_x = next_waypoint.position[0] - self._position[0]
        direction_y = next_waypoint.position[1] - self._position[1]
        direction = np.arctan2(direction_y, direction_x)
        direction = direction if direction >= 0 else direction + 2 * np.pi
        return direction

    def move(self):
        """ need to move towards the next proposed"""

        self._velocity[0] = (self.proposed_next_position[0] - self._position[0]) \
                            * self.clock.ticks_per_second / self.tickstep
        self._velocity[1] = (self.proposed_next_position[1] - self._position[1]) \
                            * self.clock.ticks_per_second / self.tickstep

        direction = np.arctan2(self.velocity[1], self._velocity[0])

        self._direction = direction if direction >= 0 else direction + 2 * np.pi
        self._position[0] = self.proposed_next_position[0]
        self._position[1] = self.proposed_next_position[1]

    def generate_base_trajectory(self):
        """ creates the optimal trajectory, given the current conditions and predictions"""
        # need to do : create an a*star generated path for the route (outputting arrays of x, y)
        self.current_own_trajectory = self.trajectory_optimisation_model.plan_trajectory()

    def generate_locally_optimised_trajectory(self):
        """
        if we detect collisions, or something else occurs, we need to replot the trajectory
        :return:
        """
        return
        obstacles = []
        for key, value in self.predicted_collisions.items():
            for i in value:
                obstacles.append(Point(i).buffer(1.0))
        future_trajectory = {key: value for key, value in self.current_own_trajectory.items() if
                             key > self.global_ticks}
        future_trajectory_optimised = self.trajectory_optimisation_model.overwrite_trajectory(obstacles,
                                                                                              future_trajectory)
        for key, value in future_trajectory_optimised.items():
            self.current_own_trajectory[key] = value

    # region Neighbour Related Model Functions
    def find_nearest_neighbour_to_location(self, location):
        nearest_distance = np.inf
        nearest_pedestrian_id = 0
        for neighbour in self.neighbours:
            if neighbour.unique_id != self.unique_id:
                distance = (location[0] - neighbour.position[0]) ** 2 + \
                           (location[1] - neighbour.position[1]) ** 2
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_pedestrian_id = neighbour.unique_id
        return {nearest_pedestrian_id: nearest_distance}

    def find_nearest_pedestrian_to_own_position(self):
        nearest_ped = self.find_nearest_neighbour_to_location(self.position)
        return nearest_ped

    def find_nearest_pedestrian_to_specific_prediction(self, prediction_override: int = None):
        """
        :param prediction_override: if you want to stipulate which particular prediction step you want to focus on
        :return: the nearest pedestrian now based on the furthest own prediction, or at a prediction_override
        """
        if prediction_override:
            if not isinstance(prediction_override, int) or prediction_override > len(
                    self.current_own_trajectory.keys()):
                raise TypeError("Prediction override should be an integer, less than the length of predictions")

            prediction_tick = list(self.current_neighbour_predictions.keys())[prediction_override]
        else:
            prediction_tick = list(self.current_neighbour_predictions.keys())[-1]

        # prediction tick is the number of ticksteps from now
        own_predicted_position = self.current_own_trajectory[(prediction_tick * self.tickstep) + self.global_ticks][
            self.unique_id]
        nearest_ped = self.find_nearest_neighbour_to_location(own_predicted_position)
        return nearest_ped

    def find_closest_approach_to_neighbours(self):
        ...
        # loop through all predictions, find the closest the agent will ever be to each of those
        # return nearest neighbour, closest approach, time to approach,

    def make_predictions_for_neighbours(self):
        """ make predictions about nearby agents"""
        # getting visible neighbours only is significantly faster
        self.get_visible_neighbours()
        if self.neighbours is not None:
            # this makes predictions n ticksteps into the future for each of the neighbours.
            self.current_neighbour_predictions = self.neighbour_prediction_model.make_predictions(self.neighbours)
        else:
            self.current_neighbour_predictions = {}

    # endregion

    # endregion
