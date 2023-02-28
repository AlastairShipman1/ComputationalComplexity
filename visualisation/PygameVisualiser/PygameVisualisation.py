import math
import numpy as np
import pygame
import sys
import pygame_menu
from visualisation.VisualisationUtils import Colours
from visualisation.PygameVisualiser.PygameRecorder import ScreenRecorder
import config
import ctypes
from visualisation.PygameVisualiser.Vehicle import EgoVehicle
from visualisation.PygameVisualiser.World import World


class Visualisation:
    def __init__(self):

        pygame.init()
        self.size = 1500, 750
        self.screen = pygame.display.set_mode(self.size)
        self.is_paused = False
        self.clock = pygame.time.Clock()

        self.world = World(self)


        if config.recording:
            self.recorder = ScreenRecorder(self.size[0], self.size[1], 10)

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
                self.recorder.capture_frame(self.screen)

        if config.recording:
            # Stop the screen recording
            self.recorder.end_recording()

        self.quit_display_and_game()

    def quit_display_and_game(self):
        pygame.display.quit()
        pygame.quit()
        sys.exit()

    def run_single_frame(self):
        dt = self.clock.tick(60)
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

        pygame.display.flip()

    def handle_events(self, event) -> bool:
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.is_paused = not self.is_paused
            elif event.key == pygame.K_ESCAPE:
                self.quit_display_and_game()

        return False

    # endregion

    # region Draw functions

    def update_world(self, dt):
        self.world.update(dt)

    def draw_world(self):
        """draw the agents here"""

        self.world.draw(self.screen)
        for agent in self.world.pygame_agents:
            agent.draw(self.screen)

    # endregion
