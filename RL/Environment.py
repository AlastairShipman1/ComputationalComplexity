import time
import random
import numpy as np
import pygame
import carla
import gym
from gym import spaces

from OwnWork.SimulationEnv.Control import KeyboardControlObject, RLControlObject
from OwnWork.SimulationEnv.IndividualSensors import CollisionSensor, LaneInvasionSensor
from OwnWork.SimulationEnv.Rendering import RenderObjectMultipleWindows
import OwnWork.SimulationEnv.Simulation_config as config
from OwnWork.SimulationEnv.Traffic import TrafficObject


class CarlaEnvironment(gym.Env):

    def __init__(self, client, world) -> None:
        self.client = client
        self.world = world
        self.ego_vehicle = None

        self.spawn_points = self.world.get_map().get_spawn_points()
        self.all_blueprints = self.world.get_blueprint_library()
        self.map = self.world.get_map()

        self.display_on = config.VISUAL_DISPLAY_ON
        self.vehicle = None

        # Objects to be kept alive
        self.traffic_object = None
        self.control_object = None
        self.render_object = None
        self.lane_invasion_sensor = None
        self.col_sensor = None

        # Two very important lists for keeping track of our actors and their observations.
        self.sensor_list = []
        self.actor_list = []

        self.original_settings = self.world.get_settings()
        self.RL_settings = self.world.get_settings()
        self.RL_settings.synchronous_mode = True  # Enables synchronous mode
        self.RL_settings.fixed_delta_seconds = 0.01
        self.RL_settings.no_rendering_mode = not config.VISUAL_DISPLAY_ON

        self.vehicle_blueprints = []
        models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']

        for vehicle in self.all_blueprints.filter('*vehicle*'):
            if any(model in vehicle.id for model in models):
                self.vehicle_blueprints.append(vehicle)

        self.ego_vehicle_blueprint = self.all_blueprints.filter(config.CAR_NAME)[0]

        self.done = False
        self.timesteps = 0
        self.ego_rotation = None
        self.ego_location = None
        self.speed = None
        self.ego_velocity = None
        self.initial_distance = np.inf
        self.parking_point = None

        self.obs = None  # np.array([positionx, positiony, vx, vy, cos_h, sin_h])
        self.goal = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.weights = np.array([1, 1, 0, 0, 0.2, 0.2], dtype=np.float32)
        self.reward = 0

        self.apply_settings()
        self.action_space = spaces.MultiDiscrete(np.array([3, 3]))
        self.observation_space = spaces.Box(low=-500, high=500, shape=(12,), dtype=np.float32)

        # # define which sensors you want to use (and also display)
        # rendered_sensors = [
        #     'sensor.camera.rgb',
        #     'sensor.lidar.ray_cast_semantic']  # "sensor.lidar.ray_cast"]#'sensor.camera.semantic_segmentation']#,'sensor.camera.instance_segmentation', 'sensor.camera.depth','sensor.camera.dvs', 'sensor.camera.optical_flow', 'sensor.lidar.ray_cast_semantic']
        # render_object = RenderObjectMultipleWindows(env,
        #                                             rendered_sensors)  # can also do RenderObjectSingleWindow to create the render in a single pygame window

    # region Gym environment methods
    def reset(self):
        try:
            self.reset_actors_and_sensors()
            self.setup_environment()

            location_offset = carla.Location(self.spawn_point.location.x + 5 * (np.random.random() - 0.5),
                                             self.spawn_point.location.y + 30 * (np.random.random() + 0.5),
                                             self.spawn_point.location.z)

            self.parking_point = carla.Transform(location_offset, self.spawn_point.rotation)
            des_pos = self.parking_point.location
            des_rot = self.parking_point.rotation.yaw
            self.goal = np.array(
                [des_pos.x, des_pos.y,
                 0, 0,
                 np.cos(des_rot), np.sin(des_rot)], dtype=np.float32)
            self.initial_distance = np.sqrt(
                (self.ego_location.x - self.goal[0]) ** 2 + (self.ego_location.y - self.goal[1]) ** 2)

            self.done = False
            self.timesteps = 0

            self.obs = self.get_observation()
        except Exception as e:
            print(e)
            print('error in reset function')
            self.reset_actors_and_sensors()

        return self.obs

    def step(self, action=None):

        try:
            self.world.tick()

            if self.display_on:
                self.render_object.update()
                self.world.debug.draw_string(self.parking_point.location, "o", draw_shadow=False,
                                             color=carla.Color(r=255, g=255, b=255), life_time=0.2,
                                             persistent_lines=True)

            self.timesteps += 1

            # Action for controlling vehicle
            if action is not None:
                self.control_object.parse_control(action)
            self.control_object.process_control()

            # Velocity, rotation, and location of the vehicle
            self.ego_velocity = self.ego_vehicle.get_velocity()
            self.ego_rotation = self.ego_vehicle.get_transform().rotation.yaw
            self.ego_location = self.ego_vehicle.get_location()

            # Rewards are given below!
            self.done = False
            self.obs = self.get_observation()

            self.reward = self.get_reward()

            # if len(self.col_sensor.history) > 0:
            #     self.col_sensor.reset_collision_history()
            #     self.reward = -10
            #     self.done=True

            if self.timesteps >= 10000:
                self.done = True

            return self.obs, self.reward, self.done, {}

        except Exception as e:
            print(f"Error in step function: {e}")
            self.quit()

    def render(self, mode="human"):
        # this is a requirement for a gym environment, but rendering is accommodated in carla- just ignore this
        pass

    # endregion

    # region helper functions

    def get_reward(self):
        int_array = []
        for i in range(6):
            int_array.append(self.obs[i] - self.goal[i])

        delta_x = self.ego_location.x - self.goal[0]
        delta_y = self.ego_location.y - self.goal[1]
        current_distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
        reward = -np.power(np.dot(np.abs(np.array(int_array)), np.array(self.weights)), .1)
        # temp_reward = -np.power(current_distance, .1)
        # self.initial_reward = -np.power(self.initial_distance, .1)
        # self.reward = (temp_reward-self.initial_reward)/self.initial_reward
        # self.reward = -np.power(current_distance/(self.initial_distance+current_distance), .3)
        return reward

    def get_observation(self):
        obs = np.array(
            [self.ego_location.x, self.ego_location.y,
             self.ego_velocity.x, self.ego_velocity.y,
             np.cos(self.ego_rotation), np.sin(self.ego_rotation),
             self.goal[0], self.goal[1],
             self.goal[2], self.goal[3],
             self.goal[4], self.goal[5]], dtype=np.float32)
        return obs

    def setup_environment(self):
        # Instantiate objects for rendering and vehicle control
        self.create_ego_vehicle()

        # this allows you to manually control, or reinforcement learning control the vehicle
        if config.INTERFACE_MODE == config.Interface_Mode.RL:
            self.control_object = RLControlObject(self)
        elif config.INTERFACE_MODE == config.Interface_Mode.Manual:
            # self.control_object = PygameControlObject(self) # this needs a pygame window
            self.control_object = KeyboardControlObject(self)

        if config.TRAFFIC_ON:
            self.traffic_object = TrafficObject(self)
            self.traffic_object.setup()

        # these are non-rendering sensors
        self.col_sensor = CollisionSensor(self)
        self.lane_invasion_sensor = LaneInvasionSensor(self)

        if self.display_on:
            # can also do RenderObjectSingleWindow to create the render all sensors in a single pygame window
            self.render_object = RenderObjectMultipleWindows(self, config.SENSOR_LIST)

    def apply_settings(self, reset=False):

        if reset:
            self.world.apply_settings(self.original_settings)
        else:
            self.world.apply_settings(self.RL_settings)

    def create_ego_vehicle(self):
        spawn_selector = 92  # random.randint(0, len(spawn_points)-1)
        self.spawn_point = self.spawn_points[spawn_selector]
        self.ego_vehicle = self.world.spawn_actor(self.ego_vehicle_blueprint, self.spawn_point)
        self.track_actor(self.ego_vehicle)
        self.ego_location = self.ego_vehicle.get_location()
        self.ego_velocity = self.ego_vehicle.get_velocity()
        self.ego_rotation = self.ego_vehicle.get_transform().rotation.yaw
        return self.ego_vehicle

    def track_actor(self, actor):
        self.actor_list.append(actor)

    def track_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def reset_actors_and_sensors(self):
        if len(self.actor_list) != 0 or len(self.sensor_list) != 0:
            for x in self.sensor_list:
                x.destroy()
            for x in self.actor_list:
                x.destroy()
            self.sensor_list.clear()
            self.actor_list.clear()

    # endregion
    def quit(self):
        self.apply_settings(True)
        self.reset_actors_and_sensors()
        if self.display_on and self.render_object is not None:
            self.render_object.destroy()
