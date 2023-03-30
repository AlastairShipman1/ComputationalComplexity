import copy

import pygame
import shapely
import random
import config
from visualisation.VisualisationUtils import Colours
from visualisation.PygameVisualiser.Vehicle import Vehicle, NormalVehicle
from visualisation.PygameVisualiser.EgoVehicle import EgoVehicle
from visualisation.PygameVisualiser.OSMData import get_edinburgh
from shapely.geometry import Polygon
from model.LocomotionUtilityClasses.Waypoint import Waypoint

"""
This is the environment
Units in metric
"""


class World:
    def __init__(self, road_network=None, optimal_route=None):

        build_1_path = config.input_image_file_path + 'Building_topdown_1.png'
        build_2_path = config.input_image_file_path + 'Building_topdown_2.png'
        build_3_path = config.input_image_file_path + 'Building_topdown_3.png'

        build_paths = [build_1_path, build_2_path, build_3_path]
        poss_rotations = [0, 90, 180, 270]

        num_buildings = 2
        num_rows = 1
        num_cols = int(num_buildings / num_rows)
        self.static_obstacles = []
        self.dynamic_obstacles = []
        self.obstacles = shapely.geometry.MultiPolygon()
        self.obstacle_locations = []
        self.obstacle_images = []
        self.original_images = []
        self.building_width = 50
        self.waypoints = []
        for i, node_id in enumerate(optimal_route):
            x= road_network.nodes[node_id]['x']
            y= road_network.nodes[node_id]['y']
            self.waypoints.append(Waypoint(i, x,y))
        # waypoints = [[25,0],[50, 50], [50, 100],[25, 125], [0, 125],[-25, 100], [-25, 25], [0, 0]  ]
        #
        # for i, val in enumerate(waypoints):
        #     #self,guid, x, y, v_x=0, v_y=0, theta=0
        #     self.waypoints.append(Waypoint(i, val[0], val[1]))

        self.pygame_agents = []
        for i in range(num_rows):
            x = 100
            y = i * 100 + 100
            for ii in range(num_cols):
                angle = random.choice(poss_rotations)
                path = random.choice(build_paths)
                building_img = pygame.image.load(path)
                rotated_image = pygame.transform.rotate(building_img, angle)
                scaled_image = pygame.transform.smoothscale(rotated_image, (self.building_width, self.building_width))
                rect = scaled_image.get_rect(topleft=(x, y))
                vertices = [(rect[0], rect[1]), (rect[0] + rect[2], rect[1]),
                       (rect[0] + rect[2], rect[1] + rect[3]), (rect[0], rect[1] + rect[3])]
                self.obstacle_images.append(scaled_image)
                self.original_images.append(scaled_image)
                self.static_obstacles.append(Polygon(vertices))
                self.obstacle_locations.append([x, y])

                v = NormalVehicle(starting_position=(x - rect[2] / 2, y * 2.5), world=self)
                self.pygame_agents.append(v)

                x += scaled_image.get_width() + 100

        self.obstacles = shapely.geometry.MultiPolygon(self.static_obstacles)
        self.ego_vehicle = EgoVehicle(world=self)
        self.ego_vehicle.world_x=self.waypoints[0].position[0]
        self.ego_vehicle.world_y=self.waypoints[0].position[1]
        self.pygame_agents.append(self.ego_vehicle)



    def update_offset(self, offset):
        for agent in self.pygame_agents:
            agent.update_offset(offset)

    def update_scale(self, updated_zoom, previous_zoom):
        ratio = updated_zoom / previous_zoom
        for i, obs in enumerate(self.obstacle_locations):
            self.obstacle_locations[i][0] *= ratio
            self.obstacle_locations[i][1] *= ratio
            self.building_width = updated_zoom * (self.obstacles.geoms[i].bounds[2] - self.obstacles.geoms[i].bounds[0])
            self.obstacle_images[i] = pygame.transform.smoothscale(self.original_images[i],
                                                                   (self.building_width, self.building_width))

        for agent in self.pygame_agents:
            agent.update_scale(updated_zoom)

    def draw(self, surface):
        for i, obs in enumerate(self.obstacle_images):
            x=self.obstacle_locations[i][0] + self.offset[0]
            y=self.obstacle_locations[i][1] + self.offset[1]
            surface.blit(obs,(x,y))
        for agent in self.pygame_agents:
            agent.draw(surface)

    def update(self, dt):
        for agent in self.pygame_agents:
            agent.update(dt)
