import pygame


class Obstacle:
    def __init__(self, parent_set, x=0, y=0, original_img=None, width=50, angle=0):
        self.original_img = original_img
        self.draw_img = original_img
        self.obstacle_set = parent_set
        self.width = width
        self.x = x
        self.y = y

        self.draw_scale = 1
        self.pixel_to_metres_ratio = 1
        self.draw_size = [self.width * self.pixel_to_metres_ratio,
                          self.width * self.pixel_to_metres_ratio]

        self.draw_offset = [0, 0]  # this is used when the background moves.
        self.img_offset = [0, 0]  # this is used to push the picture left-right or up-down, relative to the background

        if self.original_img is not None:
            self.rotated_img = pygame.transform.rotate(self.original_img, angle)
            self.draw_img = pygame.transform.smoothscale(self.rotated_img, (self.width, self.width))

            self.world_vertices = [(x, y), (x + self.width, y),
                                   (x + self.width, y + self.width), (x, y + self.width)]
            self.draw_vertices = [(x, y), (x + self.width, y),
                                  (x + self.width, y + self.width), (x, y + self.width)]
            self.draw_x = self.world_vertices[0][0] * self.draw_scale + self.draw_offset[0]
            self.draw_y = self.world_vertices[0][1] * self.draw_scale + self.draw_offset[1]

    def update_offset(self, offset):

        self.draw_offset = offset
        self.draw_x = self.world_vertices[0][0] * self.draw_scale + offset[0]
        self.draw_y = self.world_vertices[0][1] * self.draw_scale + offset[1]
        self.draw_vertices = [(self.draw_x, self.draw_y),
                              (self.draw_x + self.draw_size[0], self.draw_y),
                              (self.draw_x + self.draw_size[0], self.draw_y + self.draw_size[1]),
                              (self.draw_x, self.draw_y + self.draw_size[1])]

    def update_scale(self, scale):
        inc = scale / self.draw_scale
        self.pixel_to_metres_ratio *= inc
        self.draw_size = [self.width * self.pixel_to_metres_ratio,
                          self.width * self.pixel_to_metres_ratio]

        self.draw_x = self.world_vertices[0][0] * self.draw_scale + self.draw_offset[0]
        self.draw_y = self.world_vertices[0][1] * self.draw_scale + self.draw_offset[1]
        if self.original_img is not None:
            self.draw_img = pygame.transform.smoothscale(self.original_img, self.draw_size)
        self.draw_scale = scale

    def draw(self, surface):
        if self.draw_img is not None:
            pos = (self.draw_x - self.img_offset[0], self.draw_y - self.img_offset[1])
            surface.blit(self.draw_img, pos)

