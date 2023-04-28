import copy

import numpy as np
import pygame
import sys
from visualisation.PygameVisualiser.PygameRecorder import ScreenRecorder
import config
from visualisation.VisualisationUtils import Colours


class Visualisation:
    def __init__(self, world, background_image=None, extents=None):
        pygame.init()
        self.zoom = 1
        self.previous_pos = None
        self.is_dragging = False
        self.initial_size = 1500, 750
        self.size = 1500, 750
        self.world_size= self.size
        if extents is not None:
            self.world_size = extents[0] - extents[1], extents[2] - extents[3]
        self.offset = 0, 0
        self.screen = pygame.display.set_mode(self.size)
        self.is_paused = False
        self.clock = pygame.time.Clock()
        self.world = world
        self.tracking = False

        self.bg = None

        if background_image is not None:
            self.bg_original = pygame.image.load(background_image)
            self.bg_original = pygame.transform.rotate(self.bg_original, 180)

            self.bg = copy.deepcopy(self.bg_original)
            self.bg_original_extents = extents
            self.bg_position = [0, 0]
            # self.bg_original = pygame.image.load('./Assets/Images/bg.png')

            # self.bg_original = pygame.transform.scale(self.bg_original, self.world_size)
            self.bg = pygame.transform.smoothscale(self.bg, self.world_size)

        self.pixel_to_metre_ratio = 1  # currently 1 pixel per metre

        if config.RECORDING:
            self.recorder = ScreenRecorder(self.size[0], self.size[1], config.SIMULATION_FPS)

    # region Game logic
    def update(self):
        self.clock.tick(config.DELTA_TIME_MS)
        for event in pygame.event.get():
            finished = self.handle_events(event)
        self.draw_single_frame()



    def quit(self):
        if config.RECORDING:
            self.recorder.end_recording()
        pygame.display.quit()
        pygame.quit()

    def draw_single_frame(self):
        if self.is_paused:
            return

        self.draw_world()

        if self.tracking:
            self.recentre_on_ego_agent()

        pygame.display.flip()
        if config.RECORDING:
            self.recorder.capture_frame(self.screen)

    def handle_events(self, event) -> bool:
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.is_paused = not self.is_paused
            elif event.key == pygame.K_ESCAPE:
                self.quit()
            elif event.key == pygame.K_t:
                self.tracking = not self.tracking
            # ideally this shouldn't be here, but i think pygame.event.get purges the events, so you can only do it once?
            elif event.key == pygame.K_m:
                self.world.ego_vehicle.send_message('m')


        elif event.type == pygame.MOUSEBUTTONUP:
            self.handle_mouse_clicked(event)

        elif event.type == pygame.MOUSEMOTION:
            current_pos = pygame.mouse.get_pos()
            if pygame.mouse.get_pressed()[0]:
                self.is_dragging = True
                delta_x = current_pos[0] - self.previous_pos[0]
                delta_y = current_pos[1] - self.previous_pos[1]
                self.offset = self.offset[0] + delta_x, self.offset[1] + delta_y
                self.world.update_offset(self.offset)
            self.previous_pos = current_pos

        elif event.type == pygame.MOUSEWHEEL:
            multiplier = -np.sign(event.y)
            sc = 1 / (1.025 + multiplier * 0.225)  # cycle between 25% increase, and 80% decrease in zoom.
            zoom = self.zoom * sc
            self.offset = self.offset[0] * sc, self.offset[1] * sc
            self.world.update_scale(zoom, self.zoom)
            self.world.update_offset(self.offset)
            self.zoom = zoom
            self.pixel_to_metre_ratio = self.zoom

            # update the image
            if self.bg is not None:
                size = (int(self.bg_original.get_width() * self.zoom), int(self.bg_original.get_height() * self.zoom))
                self.bg = pygame.transform.smoothscale(self.bg_original, size)
                self.bg_position = [self.bg_position[0] * sc, self.bg_position[1] * sc]

        return False

    def handle_mouse_clicked(self, event):
        if event.button != 1:  # only use left click here. mousewheel buttons:4 or == 5:
            return
        if self.is_dragging:
            self.is_dragging = False

    def recentre_on_ego_agent(self):
        starting_pos = [self.world.ego_vehicle.draw_x, self.world.ego_vehicle.draw_y]
        delta_x = -starting_pos[0] + self.size[0] / 2
        delta_y = -starting_pos[1] + self.size[1] / 2
        self.offset = self.offset[0] + delta_x, self.offset[1] + delta_y
        self.world.update_offset(self.offset)

    # endregion

    # region Draw functions


    def draw_world(self):
        """draw the agents here"""
        # blit the background image here. after rotating 180 degrees?
        if self.bg is not None:
            self.screen.blit(self.bg, self.bg_position)
        else:
            self.screen.fill(Colours.GREY)
        self.world.draw(self.screen)

    # endregion
