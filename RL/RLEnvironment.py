import numpy as np
import gym
from gym import spaces

import config
from model.Agent.RLEgoVehicle import RLEgoVehicle


class RLEnvironment(gym.Env):
    def __init__(self, world, visualiser) -> None:

        self.visualiser = visualiser
        self.world = world
        self.world.ego_vehicle = RLEgoVehicle()
        self.world.ego_vehicle.world_x = self.world.waypoints[0].position[0]
        self.world.ego_vehicle.world_y = self.world.waypoints[0].position[1]
        self.world.pygame_agents[-1] = self.world.ego_vehicle

        self.ego_vehicle = self.world.ego_vehicle

        self.display_on = config.DISPLAY_ON
        self.done = False
        self.timesteps = 0
        self.ego_rotation = 0
        self.ego_location = [0,0]
        self.ego_velocity = [0,0]
        self.initial_distance = np.inf
        self.parking_point = [ 3365 * (np.random.random()), 165 * (np.random.random()),0]

        self.obs = None  # np.array([positionx, positiony, vx, vy, cos_h, sin_h])
        self.goal = np.array([self.parking_point[0], self.parking_point[1], 0, 0, np.cos(self.parking_point[2]), np.sin(self.parking_point[2])], dtype=np.float32)
        self.ego_vehicle.set_goal(self.parking_point)

        self.weights = np.array([1, 1, 0.05, 0.05, 0.2, 0.2], dtype=np.float32)
        self.reward = 0

        self.action_space = spaces.MultiDiscrete(np.array([3, 3]))
        self.observation_space = spaces.Box(low=-500, high=500, shape=(12,), dtype=np.float32)

    # region Gym environment methods
    def reset(self):
        try:
            self.parking_point = [ 165 * (np.random.random() - 0.5), 265 * (np.random.random()),110]
            self.goal = np.array([self.parking_point[0], self.parking_point[1], 0, 0, np.cos(self.parking_point[2]),
                                  np.sin(self.parking_point[2])], dtype=np.float32)
            self.ego_vehicle.set_goal(self.parking_point)
            self.ego_vehicle.set_position([0,0])


            self.initial_distance = np.sqrt(
                (self.ego_location[0] - self.goal[0]) ** 2 + (self.ego_location[1] - self.goal[1]) ** 2)

            self.done = False
            self.timesteps = 0
            self.ego_velocity = self.ego_vehicle.get_velocity()
            self.ego_rotation = self.ego_vehicle.direction
            self.ego_location = self.ego_vehicle.get_position()
            self.obs = self.get_observation()
        except Exception as e:
            print(f"Error in reset function: {e}")

        return self.obs

    def step(self, action=None):
        self.world.update(config.DELTA_TIME_MS) # milliseconds

        if self.display_on:
            self.visualiser.update()

        self.timesteps += 1

        # Action for controlling vehicle
        if action is not None:
            self.ego_vehicle.parse_control(action)

        # Velocity, rotation, and location of the vehicle
        self.ego_velocity = self.ego_vehicle.get_velocity()
        self.ego_rotation = self.ego_vehicle.direction
        self.ego_location = self.ego_vehicle.get_position()
        # Rewards are given below!
        self.done = False
        self.obs = self.get_observation()

        self.reward = self.get_reward()
        if self.timesteps >= 15000:
            self.done = True

        return self.obs, self.reward, self.done, {}


    def render(self, mode="human"):
        # this is a requirement for a gym environment, but rendering is accommodated in pygame- just ignore this
        pass

    # endregion

    # region helper functions

    def get_reward(self):
        # obs: np.array([positionx, positiony, vx, vy, cos_h, sin_h])
        # goal: np.array([positionx, positiony, vx, vy, cos_h, sin_h])

        # are you moving towards the point?
        # minimum manouvres to get to the point.
        # time to closest approach?

        # are you near the point?




        int_array = []
        for i in range(6):
            int_array.append(self.obs[i] - self.goal[i])

        reward = -np.power(np.dot(np.abs(np.array(int_array)), np.array(self.weights)), 5)
        return reward

    def get_observation(self):
        obs = np.array(
            [self.ego_location[0], self.ego_location[1],
             self.ego_velocity[0], self.ego_velocity[1],
             np.cos(self.ego_rotation), np.sin(self.ego_rotation),
             self.goal[0], self.goal[1],
             self.goal[2], self.goal[3],
             self.goal[4], self.goal[5]], dtype=np.float32)
        return obs


    # endregion
