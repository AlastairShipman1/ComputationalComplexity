import copy
import numpy as np
import pygame
import sys
from visualisation.PygameVisualiser.PygameRecorder import ScreenRecorder
import config
import os

class Visualisation:
    def __init__(self, world, background_image=None, extents=None):
        pygame.init()
        self.zoom = 1
        self.previous_pos = None
        self.is_dragging = False
        self.initial_size = 1500, 750
        self.display_size = 1500, 750
        self.world_size = extents[0]-extents[1], extents[2]-extents[3]
        self.offset = 0, 0
        self.screen = pygame.display.set_mode(self.display_size)
        self.is_paused = False
        self.clock = pygame.time.Clock()
        self.world = world
        self.tracking = False

        if background_image is not None:
            self.bg_original = pygame.image.load(background_image)
        elif os.path.exists(config.input_image_file_path+"bg_low_qual.png"):
            self.bg_original = pygame.image.load(config.input_image_file_path + 'bg_low_qual.png')
        else:
            raise Exception("need a background image, or refactor to remove requirement")

        self.bg_original = pygame.transform.rotate(self.bg_original, 180)

        self.bg = pygame.transform.scale(self.bg_original, self.bg_original.get_size())
        self.bg_original_extents=extents
        self.bg_position = self.world.waypoints[0].position

        # self.bg_original = pygame.transform.scale(self.bg_original, self.world_size)
        # self.bg = pygame.transform.smoothscale(self.bg, self.world_size)

        self.pixel_to_metre_ratio = 1 # currently 1 pixel per metre

        if config.recording:
            self.recorder = ScreenRecorder(self.display_size[0], self.display_size[1], config.simulation_fps)

    # region Game logic
    def run(self):
        finished = False
        while True:
            # check events
            for event in pygame.event.get():
                finished = self.handle_events(event)
            if finished:
                break

            self.run_single_frame()

        if config.recording:
            # Stop the screen recording
            self.recorder.end_recording()

        self.quit_display_and_game()

    def quit_display_and_game(self):
        pygame.display.quit()
        pygame.quit()
        sys.exit()

    def run_single_frame(self):
        dt = self.clock.tick(config.simulation_fps)

        if self.is_paused:
            return

        keys = pygame.key.get_pressed()  # checking pressed keys

        if keys[pygame.K_UP]:
            self.world.ego_vehicle.send_message("u")
        if keys[pygame.K_DOWN]:
            self.world.ego_vehicle.send_message("d")
        if keys[pygame.K_LEFT]:
            self.world.ego_vehicle.send_message("l")
        if keys[pygame.K_RIGHT]:
            self.world.ego_vehicle.send_message("r")

        self.update_world(dt)
        self.draw_world()

        if self.tracking:
            self.recentre_on_ego_agent()

        pygame.display.flip()
        if config.recording:
            self.recorder.capture_frame(self.screen)

    def handle_events(self, event) -> bool:
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.is_paused = not self.is_paused
            elif event.key == pygame.K_ESCAPE:
                self.quit_display_and_game()
            elif event.key == pygame.K_t:
                self.tracking = not self.tracking

        elif event.type == pygame.MOUSEBUTTONUP:
            self.handle_mouse_clicked(event)

        elif event.type == pygame.MOUSEMOTION:
            self.handle_mousemotion(pygame.mouse.get_pos())

        elif event.type == pygame.MOUSEWHEEL:
            self.handle_mousewheel(-np.sign(event.y))

        return False

    def handle_mousemotion(self, current_pos=(0,0)):
        if pygame.mouse.get_pressed()[0]:
            self.is_dragging = True
            delta_x = current_pos[0] - self.previous_pos[0]
            delta_y = current_pos[1] - self.previous_pos[1]
            self.offset = self.offset[0] + delta_x, self.offset[1] + delta_y
            for agent in self.world.pygame_agents:
                agent.update_offset(self.offset)
        self.previous_pos = current_pos

    def handle_mousewheel(self, multiplier):
        sc = 1 / (1.025 + multiplier * 0.225)  # cycle between 25% increase, and 80% decrease in zoom.
        zoom = self.zoom * sc
        self.offset = self.offset[0] * sc, self.offset[1] * sc
        self.world.update_scale(zoom, self.zoom)
        self.zoom = zoom
        self.pixel_to_metre_ratio = self.zoom

        # update the image
        size = (int(self.bg_original.get_width() * self.zoom), int(self.bg_original.get_height() * self.zoom))
        self.bg = pygame.transform.smoothscale(self.bg_original, size)
        self.bg_position = [self.bg_position[0] * sc, self.bg_position[1] * sc]

    def handle_mouse_clicked(self, event):
        if event.button != 1:  # only use left click here. mousewheel buttons:4 or == 5:
            return
        if self.is_dragging:
            self.is_dragging = False

    def recentre_on_ego_agent(self):
        starting_pos = [self.world.ego_vehicle.draw_x, self.world.ego_vehicle.draw_y]
        delta_x = -starting_pos[0] + self.display_size[0] / 2
        delta_y = -starting_pos[1] + self.display_size[1] / 2
        self.offset = self.offset[0] + delta_x, self.offset[1] + delta_y
        self.update_offset(self.offset)

    def update_offset(self, offset):
        for agent in self.world.pygame_agents:
            agent.update_offset(offset)

    # endregion

    # region Draw functions

    def update_world(self, dt):
        self.world.update(dt)

    def draw_world(self):
        """draw the agents here"""
        # blit the background image here. after rotating 180 degrees?
        self.screen.blit(self.bg, self.offset)

        for i, obs in enumerate(self.world.obstacle_images):
            x = self.world.obstacle_locations[i][0] + self.offset[0]
            y = self.world.obstacle_locations[i][1] + self.offset[1]
            self.screen.blit(obs, (x, y))
        for agent in self.world.pygame_agents:
            agent.draw(self.screen)


    # endregion
