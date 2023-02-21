import math
import numpy as np
import pygame
import sys
import pygame_menu
from visualisation.VisualisationUtils import Colours
from visualisation.PygameVisualiser.PygameRecorder import ScreenRecorder
import config
import ctypes


class Visualisation:
    def __init__(self):

        pygame.init()

        # fix the axis of the display to be x% of appropriate dimension
        self.size = 1500, 750

        self.ego_vehicle = None
        self.pygame_agents = []
        self.obstacles = []

        # now that the size has been appropriately set, we can move onto setting up the surfaces
        self.screen = pygame.display.set_mode(self.size, pygame.RESIZABLE)
        self.is_paused = False
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
        if self.is_paused:
            return

        self.previous_pos = pygame.mouse.get_pos()
        self.screen.fill(Colours.BLUE)
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

    def draw_agents(self):
        """draw the agents here"""
        for agent in self.pygame_agents:
            agent.draw(self.tickstamps[self.time_iterator], self.screen)
            if agent.showing_predictions:
                agent.draw_predictions(self.tickstamps[self.time_iterator], self.screen)
                agent.draw_planned_trajectory(self.tickstamps[self.time_iterator], self.screen)

    # endregion

