import pygame
import shapely
import random
import config
from visualisation.VisualisationUtils import Colours
from visualisation.PygameVisualiser.Vehicle import EgoVehicle, Vehicle, NormalVehicle
from shapely.geometry import Polygon

"""
This is the environment
Units should be in metric, including metres

"""

class World():
    def __init__(self, visualiser=None):
        if visualiser is not None:
            self.initial_size = visualiser.initial_size
        else:
            self.initial_size = (1000, 1000)
        self.visualiser=visualiser

        build_1_path = config.input_image_file_path + 'Building_topdown_1.png'
        build_2_path = config.input_image_file_path + 'Building_topdown_2.png'
        build_3_path = config.input_image_file_path + 'Building_topdown_3.png'

        build_paths = [build_1_path, build_2_path, build_3_path]
        poss_rotations = [0, 90, 180, 270]

        num_buildings = 2
        num_rows = 1
        num_cols = int(num_buildings / num_rows)
        self.obstacles = []
        self.obstacle_locations = []
        self.obstacle_images = []

        self.pygame_agents = []
        for i in range(num_rows):
            x = 100
            y = i * 100 + 100
            for ii in range(num_cols):
                angle = random.choice(poss_rotations)
                path = random.choice(build_paths)
                building_img = pygame.image.load(path)
                rotated_image = pygame.transform.rotate(building_img, angle)
                scaled_image = pygame.transform.smoothscale(rotated_image, (50,50))
                rect=scaled_image.get_rect(topleft=(x,y))
                pos=[(rect[0], rect[1]),(rect[0]+rect[2], rect[1]),
                     (rect[0]+rect[2], rect[1]+rect[3]), (rect[0], rect[1]+rect[3]) ]
                self.obstacle_images.append(scaled_image)
                self.obstacles.append(Polygon(pos))
                self.obstacle_locations.append([x,y])

                v = NormalVehicle(starting_position=(x-rect[2]/2, y*2.5))
                self.pygame_agents.append(v)

                x += scaled_image.get_width() + 100

        self.obstacles = shapely.geometry.MultiPolygon(self.obstacles)
        self.ego_vehicle = EgoVehicle(self)


        self.pygame_agents.append(self.ego_vehicle)

    def update_offset(self):
        ...

    def update_scale(self):
        ...

    def draw(self, surface):
        surface.fill(Colours.GREY)
        for i, obs in enumerate(self.obstacle_images):
            surface.blit(obs, self.obstacle_locations[i])

    def update(self, dt):
        for agent in self.pygame_agents:
            agent.update(dt)
            if(agent.y<0):
                agent.y=self.initial_size[1]