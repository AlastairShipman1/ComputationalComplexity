import numpy as np
import pygame
import sys
from visualisation.PygameVisualiser.PygameRecorder import ScreenRecorder
import config
from visualisation.PygameVisualiser.World import World


class Visualisation:
    def __init__(self):
        pygame.init()

        self.zoom = 1
        self.previous_pos = None
        self.is_dragging = False
        self.initial_size = 1500, 750
        self.size = 1500, 750
        self.offset = 0, 0
        self.screen = pygame.display.set_mode(self.size)
        self.is_paused = False
        self.clock = pygame.time.Clock()
        self.world = World(self)

        if config.recording:
            self.recorder = ScreenRecorder(self.size[0], self.size[1], config.simulation_fps)

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

        pygame.display.flip()

    def handle_events(self, event) -> bool:
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.is_paused = not self.is_paused
            elif event.key == pygame.K_ESCAPE:
                self.quit_display_and_game()

        elif event.type == pygame.MOUSEBUTTONUP:
            self.handle_mouse_clicked(event)

        elif event.type == pygame.MOUSEMOTION:
            current_pos = pygame.mouse.get_pos()
            if pygame.mouse.get_pressed()[0]:
                self.is_dragging = True
                delta_x = current_pos[0] - self.previous_pos[0]
                delta_y = current_pos[1] - self.previous_pos[1]
                self.offset = self.offset[0] + delta_x, self.offset[1] + delta_y
                for agent in self.world.pygame_agents:
                    agent.update_offset(self.offset)
            self.previous_pos = current_pos

        elif event.type == pygame.MOUSEWHEEL:
            multiplier = -np.sign(event.y)
            sc = 1 / (1.025 + multiplier * 0.225)  # cycle between 25% increase, and 80% decrease in zoom.
            self.zoom *= sc
            size = (int(self.initial_size[0] * self.zoom), int(self.initial_size[1] * self.zoom))
            self.offset = self.offset[0] * sc, self.offset[1] * sc
            for agent in self.world.pygame_agents:
                agent.update_scale(self.zoom, sc)
                agent.update_offset(self.offset)

        return False

    def handle_mouse_clicked(self, event):
        if event.button != 1:  # only use left click here. mousewheel buttons:4 or == 5:
            return
        if self.is_dragging:
            self.is_dragging = False


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
