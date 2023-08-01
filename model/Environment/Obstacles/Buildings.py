import pygame

from model.Environment.Obstacles.Static_Obstacles import Obstacle


class Building(Obstacle):
    def __init__(self, parent_set, x, y, path, angle):
        self.width = 50
        self.building_img = pygame.image.load(path)
        self.rotated_img = pygame.transform.rotate(self.building_img, angle)
        self.draw_img = pygame.transform.smoothscale(self.rotated_img, (self.width, self.width))
        rect = self.draw_img.get_rect(topleft=(x, y))
        self.world_vertices = [(rect[0], rect[1]), (rect[0] + rect[2], rect[1]),
                               (rect[0] + rect[2], rect[1] + rect[3]), (rect[0], rect[1] + rect[3])]
        self.draw_vertices = [(rect[0], rect[1]), (rect[0] + rect[2], rect[1]),
                              (rect[0] + rect[2], rect[1] + rect[3]), (rect[0], rect[1] + rect[3])]
        super().__init__(parent_set, x, y, self.draw_img)
