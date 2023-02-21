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
    def __init__(self, pygame_agents, clock):
        pygame.init()

        # fix the axis of the display to be x% of appropriate dimension
        screensize = 1500, 750

        if sys.platform == "win32":
            # windows
            user32 = ctypes.windll.user32
            screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
        elif sys.platform == "darwin":
            #screensize = NSScreen.mainScreen().frame().size.width, NSScreen.mainScreen().frame().size.height
            pass
        else:
            pass
        screen_proportion = config.visualisation_metrics["screen_prop"]
        target_height = int(screen_proportion * screensize[1])
        target_width = int(screen_proportion * screensize[0])

        # get background image
        # self.bg = pygame.image.load('./assets/Images/stock_city_612x612.jpg')
        # self.bg_original = pygame.image.load('./assets/Images/stock_city_612x612.jpg')
        self.bg_original = pygame.image.load('./assets/Images/bg.png')

        initial_aspect_ratio = self.bg_original.get_width() / self.bg_original.get_height()

        if screensize[0] * target_height / screensize[1] > screensize[0]:
            self.size = target_width, int(target_width / initial_aspect_ratio)
        else:
            self.size = int(target_height * initial_aspect_ratio), target_height

        self.bg_original = pygame.transform.scale(self.bg_original, self.size)
        self.bg = pygame.transform.scale(self.bg_original, self.size)

        self.bg_position = (0, 0)
        self.zoom = 1

        # now that the size has been appropriately set, we can move onto setting up the surfaces

        fontsize = 20
        self.myfont = pygame.font.SysFont('arial', fontsize)
        self.screen = pygame.display.set_mode(self.size, pygame.RESIZABLE)

        self.timestamp_screen = pygame.Surface((200, 20), pygame.SRCALPHA)
        self.frame_rate_screen = pygame.Surface((200, 20), pygame.SRCALPHA)

        self.toggle_framerate_event = pygame.USEREVENT + 1

        self.time_value = 0
        self.run_speed = 1
        self.target_frame_time_millisecs = clock.tickstep
        self.tick_time_conversion_rate = clock.ticks_per_second
        self.now = pygame.time.get_ticks()
        self.tickstamps = set([])
        self.pygame_agents = pygame_agents
        self.is_paused = False
        self.displaying_framerate = False
        self.time_iterator = 0

        self.is_dragging = False
        self.previous_pos = (0, 0)

        theme = pygame_menu.themes.Theme(title_font=self.myfont)
        theme.widget_selection_effect = \
            pygame_menu.widgets.HighlightSelection(1, 0, 0).set_color((120, 120, 120))
        self.menu = pygame_menu.Menu('Register data', 200, 200, position=(10, 50), mouse_motion_selection=True,
                                     theme=theme)
        self.menu.add.button("Do nothing", self.menu_do_nothing, font_size=fontsize)
        self.menu.add.button('Also do nothing', self.menu_do_nothing, font_size=fontsize)
        self.menu.add.button('Niente', self.menu_do_nothing, font_size=fontsize)
        self.menu.add.button('Nada', self.menu_do_nothing, font_size=fontsize)
        self.menu.disable()

        # now scale the agent trajectories given the monitor size, and get a list of all timestamps
        for agent in pygame_agents:
            agent.setup(self.size)
            tickstamps = agent.raw_actual_trajectory.keys()
            for i in tickstamps:
                self.tickstamps.add(i)
        # now sort the set.
        self.tickstamps = sorted(self.tickstamps)
        self.max_tick = max(self.tickstamps)
        self.min_tick = min(self.tickstamps)

        if config.visualisation_metrics["recording"]:
            self.recorder = ScreenRecorder(self.size[0], self.size[1], 10)

    # region Game logic
    def run(self):
        while True:
            # check events
            for event in pygame.event.get():
                self.handle_events(event)
                if self.menu.is_enabled():
                    self.menu.update([event])

            self.run_single_frame()

            if config.visualisation_metrics["recording"]:
                self.recorder.capture_frame(self.screen)

            # cycle through time.make sure you don't keep cycling through
            if not self.is_paused:
                self.time_iterator += int(np.sign(self.run_speed))
            if self.time_iterator >= len(self.tickstamps) or self.time_iterator < 0:
                break

        if config.visualisation_metrics["recording"]:
            # Stop the screen recording
            self.recorder.end_recording()

        self.quit_display_and_game()

    def quit_display_and_game(self):
        pygame.display.quit()
        pygame.quit()
        sys.exit()

    def run_single_frame(self):
        self.previous_pos = pygame.mouse.get_pos()

        self.screen.fill(Colours.CLEAR)
        self.screen.blit(self.bg, self.bg_position)
        self.draw_agents()
        self.draw_timestamp()
        self.draw_framerate()
        if self.menu.is_enabled():
            self.menu.draw(self.screen)

        pygame.display.flip()

        if self.run_speed != 0 and not self.is_paused:
            self.now = pygame.time.get_ticks()
            frame_vs_real_time_diff = (self.target_frame_time_millisecs / abs(self.run_speed)) - (
                    pygame.time.get_ticks() - self.now)
            pygame.time.delay(int(frame_vs_real_time_diff))

    def handle_events(self, event):
        if event.type == pygame.QUIT:
            self.quit_display_and_game()

        elif event.type == pygame.MOUSEBUTTONUP:
            self.handle_mouse_clicked(event)

        elif event.type == pygame.MOUSEMOTION:
            if pygame.mouse.get_pressed()[0]:
                self.is_dragging = True
                current_pos = pygame.mouse.get_pos()
                delta_x = current_pos[0] - self.previous_pos[0]
                delta_y = current_pos[1] - self.previous_pos[1]
                self.bg_position = (self.bg_position[0] + delta_x, self.bg_position[1] + delta_y)
                self.previous_pos = current_pos
                for agent in self.pygame_agents:
                    agent.update_offset(self.bg_position)

        elif event.type == pygame.MOUSEWHEEL:
            multiplier = -np.sign(event.y)
            sc = 1 / (1.025 + multiplier * 0.225)  # cycle between 25% increase, and 80% decrease in zoom.
            self.zoom *= sc
            size = (int(self.bg_original.get_width() * self.zoom), int(self.bg_original.get_height() * self.zoom))
            self.bg = pygame.transform.smoothscale(self.bg_original, size)
            self.bg_position = [self.bg_position[0] * sc, self.bg_position[1] * sc]
            for agent in self.pygame_agents:
                agent.update_scale(self.zoom, sc)
                agent.update_offset(self.bg_position)

        elif event.type == self.toggle_framerate_event:
            self.toggle_framerate_display()

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.is_paused = not self.is_paused

            elif event.key == pygame.K_ESCAPE:
                self.quit_display_and_game()

            elif event.key == pygame.K_DOWN:
                self.update_frame_rate(-0.5)

            elif event.key == pygame.K_UP:
                self.update_frame_rate(0.5)

            elif event.key == pygame.K_LEFT:
                if self.run_speed > 0:
                    self.run_speed *= -1
                    self.update_frame_rate()

            elif event.key == pygame.K_RIGHT:
                if self.run_speed < 0:
                    self.run_speed *= -1
                    self.update_frame_rate()

    def handle_framerate_display(self):
        self.run_single_frame()
        self.frame_rate_screen.fill(Colours.CLEAR)
        rate_text = self.myfont.render(f"{self.run_speed * 100:.0f}%", True, Colours.BLUE)
        self.frame_rate_screen.blit(rate_text, (0, 0))

    def handle_mouse_clicked(self, event):
        if event.button != 1:  # only use left click here. mousewheel buttons:4 or == 5:
            return

        if self.menu.is_enabled():
            updated = self.menu.update([event])
            if not updated:
                self.menu.disable()
                self.is_paused = False
            return

        if self.is_dragging:
            self.is_dragging = False
        elif not self.menu.is_enabled():
            self.menu.enable()
            self.set_menu_position()
            self.is_paused = True

    def set_menu_position(self):
        pos = pygame.mouse.get_pos()
        shift_x, shift_y = 0, 0
        # if pos is too close to the edge, shift to a different corner.
        if pos[0] + self.menu.get_width() > self.size[0]:
            shift_x = self.menu.get_width()
        if pos[1] + self.menu.get_height() > self.size[1]:
            shift_y = self.menu.get_height()
        pos = [pos[0] - shift_x, pos[1] - shift_y]
        self.menu.set_absolute_position(pos[0], pos[1])

    # endregion

    # region Menu functions

    def menu_do_nothing(self):
        self.is_paused = False
        self.menu.disable()

    # endregion

    # region Draw functions
    def draw_framerate(self):
        if self.displaying_framerate:
            self.screen.blit(self.frame_rate_screen, (self.bg_position[0], self.bg_position[1] + 40))

    def draw_agents(self):
        """draw the agents here"""
        for agent in self.pygame_agents:
            agent.draw(self.tickstamps[self.time_iterator], self.screen)
            if agent.showing_predictions:
                agent.draw_predictions(self.tickstamps[self.time_iterator], self.screen)
                agent.draw_planned_trajectory(self.tickstamps[self.time_iterator], self.screen)

    def draw_timestamp(self):
        self.timestamp_screen.fill(Colours.CLEAR)
        self.time_value = self.tickstamps[self.time_iterator] / self.tick_time_conversion_rate
        timetext = self.myfont.render(f"{self.time_value}", True, Colours.BLUE)
        self.timestamp_screen.blit(timetext, (0, 0))
        self.screen.blit(self.timestamp_screen, self.bg_position)

    # endregion

    # region Utility functions
    def update_frame_rate(self, delta_frame_rate: float = 0):
        # create event to toggle frame rate display
        proposed_framerate = self.run_speed + delta_frame_rate
        pygame.time.set_timer(self.toggle_framerate_event, 1500, loops=1)
        if self.displaying_framerate:
            self.toggle_framerate_display()
        self.run_speed = proposed_framerate
        self.toggle_framerate_display()

    def toggle_framerate_display(self):
        self.handle_framerate_display()
        self.displaying_framerate = not self.displaying_framerate

    def toggle_highlight_nearest_agent_to_mouse(self):
        # find nearest agent, set to highlighting
        nearest_agent = self.get_nearest_agent_to_mouse()
        if nearest_agent is not None:
            nearest_agent.toggle_highlighting()

    def get_nearest_agent_to_mouse(self):
        mouse_pos = pygame.mouse.get_pos()
        nearest_agent = None
        dist = math.inf
        for agent in self.pygame_agents:
            agent_dist = (agent.x - mouse_pos[0]) ** 2 + (agent.y - mouse_pos[1]) ** 2
            if agent_dist < dist:
                nearest_agent = agent
                dist = agent_dist
        return nearest_agent

    # endregion
