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

class Visualisation:
    def __init__(self):

        pygame.init()

        self.size = 1500, 750

        self.ego_vehicle = EgoVehicle()
        self.pygame_agents = [self.ego_vehicle]
        self.obstacles = []

        # now that the size has been appropriately set, we can move onto setting up the surfaces
        self.screen = pygame.display.set_mode(self.size)
        self.bg = pygame.image.load(config.input_image_file_path + 'citymap.png')
        self.bg=pygame.transform.scale(self.bg, self.size)
        self.screen.blit(self.bg, (0,0))
        self.is_paused = False
        self.clock = pygame.time.Clock()
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
        self.screen.blit(self.bg, (0, 0))

        self.previous_pos = pygame.mouse.get_pos()
        self.update_agents(dt)
        self.draw_agents()

        pygame.display.flip()

    def handle_events(self, event) -> bool:
        if event.type == pygame.QUIT:
            return True


        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.is_paused = not self.is_paused

            elif event.key == pygame.K_ESCAPE:
                self.quit_display_and_game()

            elif event.key == pygame.K_DOWN:
                self.ego_vehicle.send_message("d")
            elif event.key == pygame.K_UP:
                self.ego_vehicle.send_message("u")

            elif event.key == pygame.K_LEFT:
                self.ego_vehicle.send_message("l")

            elif event.key == pygame.K_RIGHT:
                self.ego_vehicle.send_message("r")

        return False


    # endregion

    # region Draw functions

    def update_agents(self, dt):
        for agent in self.pygame_agents:
            agent.update(dt)

    def draw_agents(self):
        """draw the agents here"""
        for agent in self.pygame_agents:
            agent.draw(self.screen)

    # endregion

