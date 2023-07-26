import random
from shapely import Polygon
import config
import pygame


class Obstacle_set:
    def __init__(self):
        num_buildings = config.OBSTACLE_NUMBER
        num_rows = 1
        num_cols = int(num_buildings / num_rows)

        build_1_path = config.INPUT_IMAGE_FP + '/Building_topdown_1.png'
        build_2_path = config.INPUT_IMAGE_FP + '/Building_topdown_2.png'
        build_3_path = config.INPUT_IMAGE_FP + '/Building_topdown_3.png'

        build_paths = [build_1_path, build_2_path, build_3_path]
        poss_rotations = [0, 90, 180, 270]

        self.obstacles = []
        self.obstacle_images = []
        self.draw_scale = 1
        self.pixel_to_metres_ratio = 1

        for i in range(num_rows):
            x = 100
            y = i * 100 + 100
            for ii in range(num_cols):
                angle = random.choice(poss_rotations)
                path = random.choice(build_paths)
                obstacle = Obstacle(self, x, y, path, angle)
                x += obstacle.scaled_image.get_width() + 100
                self.obstacles.append(obstacle)

    def update_offset(self, offset):
        for obs in self.obstacles:
            obs.update_offset(offset)

    def update_scale(self, scale):
        for obs in self.obstacles:
            obs.update_scale(scale)

    def draw(self, surface):
        for obs in enumerate(self.obstacles):
            obs.draw(surface)



class Obstacle:
    def __init__(self, parent_set, x=0, y=0, original_img=None, width=50, angle=0):
        self.original_img = original_img
        self.obstacle_set = parent_set
        self.width = width
        self.x = x
        self.y = y
        self.draw_scale = 1
        self.pixel_to_metres_ratio = 1

        if self.original_img is not None:
            self.rotated_image = pygame.transform.rotate(self.original_img, angle)
            self.scaled_image = pygame.transform.smoothscale(self.rotated_image, (self.width, self.width))

            self.world_vertices = [(x, y), (x + self.width, y),
                                   (x + self.width, y + self.width), (x, y +self.width)]
            self.draw_vertices = [(x, y), (x + self.width, y),
                                  (x + self.width, y + self.width), (x, y + self.width)]

    def update_offset(self, offset):
        self.draw_offset = offset
        self.draw_x = self.world_vertices[0][0] * self.draw_scale + offset[0]
        self.draw_y = self.world_vertices[0][1] * self.draw_scale + offset[1]
        width = self.width * self.draw_scale
        self.draw_vertices = [(self.draw_x, self.draw_y), (self.draw_x + width, self.draw_y),
                              (self.draw_x + width, self.draw_y + width), (self.draw_x, self.draw_y + width)]

    def update_scale(self, scale):
        inc = scale / self.draw_scale
        self.pixel_to_metres_ratio *= inc
        self.draw_size = [self.width * self.pixel_to_metres_ratio,
                          self.width * self.pixel_to_metres_ratio]
        self.draw_scale = scale

    def draw(self, surface):
        ...


class Building(Obstacle):
    def __init__(self, parent_set, x, y, path):
        super().__init__(parent_set, x, y, path)
        self.width = 50
        self.building_img = pygame.image.load(path)
        self.rotated_image = pygame.transform.rotate(self.building_img, angle)
        self.scaled_image = pygame.transform.smoothscale(self.rotated_image, (self.width, self.width))
        rect = self.scaled_image.get_rect(topleft=(x, y))
        self.world_vertices = [(rect[0], rect[1]), (rect[0] + rect[2], rect[1]),
                               (rect[0] + rect[2], rect[1] + rect[3]), (rect[0], rect[1] + rect[3])]
        self.draw_vertices = [(rect[0], rect[1]), (rect[0] + rect[2], rect[1]),
                              (rect[0] + rect[2], rect[1] + rect[3]), (rect[0], rect[1] + rect[3])]
