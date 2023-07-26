import random
import config
import pygame

class Obstacle_set:
    def __init__(self, number):
        self.number = number
        build_1_path = config.INPUT_IMAGE_FP + '/Building_topdown_1.png'
        build_2_path = config.INPUT_IMAGE_FP + '/Building_topdown_2.png'
        build_3_path = config.INPUT_IMAGE_FP + '/Building_topdown_3.png'

        build_paths = [build_1_path, build_2_path, build_3_path]
        poss_rotations = [0, 90, 180, 270]

        self.obstacles =[]
        for i in range(number):
            angle = random.choice(poss_rotations)
            path = random.choice(build_paths)
            self.obstacles.append(Obstacle(path, angle))

class Obstacle:

    def __init__(self, path, angle=0):
        self.width = 50
        building_img = pygame.image.load(path)
        rotated_image = pygame.transform.rotate(building_img, angle)
        scaled_image = pygame.transform.smoothscale(rotated_image, (self.width, self.width))
        rect = scaled_image.get_rect(topleft=(x, y))
        vertices = [(rect[0], rect[1]), (rect[0] + rect[2], rect[1]),
                    (rect[0] + rect[2], rect[1] + rect[3]), (rect[0], rect[1] + rect[3])]
        self.obstacle_images.append(scaled_image)
        self.original_images.append(scaled_image)
        self.static_obstacles.append(Polygon(vertices))
        self.obstacle_locations.append([x, y])



