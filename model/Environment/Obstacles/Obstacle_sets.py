import random

import shapely
from shapely import Polygon

import config
from model.Environment.Obstacles.Buildings import Building


class Obstacle_set:
    def __init__(self):
        self.obstacles = []
        self.obstacles_polygon = shapely.geometry.MultiPolygon()

    def create_obstacles(self):
        num_buildings = config.BUILDING_NUMBER
        num_rows = 1
        num_cols = int(num_buildings / num_rows)

        build_1_path = config.INPUT_IMAGE_FP + '/Building_topdown_1.png'
        build_2_path = config.INPUT_IMAGE_FP + '/Building_topdown_2.png'
        build_3_path = config.INPUT_IMAGE_FP + '/Building_topdown_3.png'

        build_paths = [build_1_path, build_2_path, build_3_path]
        poss_rotations = [0, 90, 180, 270]

        world_obstacles = []

        for i in range(num_rows):
            x = 100
            y = i * 100 + 100
            for ii in range(num_cols):
                angle = random.choice(poss_rotations)
                path = random.choice(build_paths)
                obstacle = Building(self, x, y, path, angle)
                x += obstacle.draw_img.get_width() + 100
                self.obstacles.append(obstacle)
                world_obstacles.append(Polygon(obstacle.world_vertices))

        self.obstacles_polygon = shapely.geometry.MultiPolygon(world_obstacles)
