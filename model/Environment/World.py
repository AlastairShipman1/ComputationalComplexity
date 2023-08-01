import numpy as np

import config
from model.Agent.Vehicle import NormalVehicle
from model.Agent.EgoVehicle import EgoVehicle
from model.ModelUtils import Waypoint
from model.Environment.Obstacles import Obstacle_sets
"""
This is the environment
Units in metric
angles in radians
"""


class World:
    def __init__(self, road_network=None, optimal_route=None):
        self.draw_offset = 0, 0
        self.waypoints = []
        self.t = 0

        if road_network is not None and optimal_route is not None:
            for i, node_id in enumerate(optimal_route):
                x = road_network.nodes[node_id]['x']
                y = road_network.nodes[node_id]['y']
                self.waypoints.append(Waypoint(i, x, y))
        else:
            waypoints = [[25, 0], [50, 50], [50, 100], [25, 125], [0, 125], [-25, 100], [-25, 25], [0, 0]]
            for i, val in enumerate(waypoints):
                # self,guid, x, y, v_x=0, v_y=0, theta=0
                self.waypoints.append(Waypoint(i, val[0], val[1], theta=4 * np.pi * (np.random.random() - 0.5)))

        self.all_agents = []
        self.building_width = 50
        self.obstacle_set = Obstacle_sets.Obstacle_set()
        if config.OBSTACLES_ON:
            self.create_vehicles()
            self.obstacle_set.create_obstacles()

        self.ego_vehicle = EgoVehicle(world=self)
        self.ego_vehicle.world_x = self.waypoints[0].position[0]
        self.ego_vehicle.world_y = self.waypoints[0].position[1]
        self.all_agents.append(self.ego_vehicle)

    def create_vehicles(self):

        num_buildings = config.BUILDING_NUMBER
        num_rows = 1
        num_cols = int(num_buildings / num_rows)
        for i in range(num_rows):
            x = 100
            y = i * 100 + 100
            for ii in range(num_cols):
                v = NormalVehicle(starting_position=(x - self.building_width / 2, y * 2.5), world=self)
                self.all_agents.append(v)
                x += self.building_width + 100

    def update_offset(self, offset):
        self.draw_offset = offset
        for obs in self.obstacle_set.obstacles:
            obs.update_offset(offset)
        for agent in self.all_agents:
            agent.update_offset(offset)

    def update_scale(self, updated_zoom):
        for obs in self.obstacle_set.obstacles:
            obs.update_scale(updated_zoom)

        for agent in self.all_agents:
            agent.update_scale(updated_zoom)

    def draw(self, surface):
        for obs in self.obstacle_set.obstacles:
            obs.draw(surface)

        for agent in self.all_agents:
            agent.draw(surface)

    def update(self, dt):
        self.t += dt
        for agent in self.all_agents:
            agent.update(dt)
