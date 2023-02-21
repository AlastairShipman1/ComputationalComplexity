import pygame
import queue
from visualisation.VisualisationUtils import Colours
import shapely
import math


class PygameAgent(pygame.sprite.Sprite):

    # region Initialising functions
    def __init__(self, agent):
        self.model_agent = agent
        self.create_containers()
        self.create_properties()

        self.set_raw_trajectory(self.model_agent.past_trajectory)
        self.set_raw_predictions(self.model_agent.all_predictions)
        self.set_planned_trajectory(self.model_agent.current_own_trajectory)

    def create_containers(self):
        self.raw_actual_trajectory = {}
        self.standardised_actual_trajectory = {}
        self.raw_planned_trajectory = {}
        self.standardised_planned_trajectory = {}

        self.extremes = {}

        self.raw_predictions = {}
        self.standardised_predictions = {}
        self.current_predictions = {}

        self.draw_offset = [0, 0]
        self.draw_scale = 1

    def create_properties(self):
        self.trace = queue.Queue(30)
        #### initialise position and direction ####
        self.x = 0
        self.y = 0

        # here for info only- these get overwritten when predictions are initialised.
        self.previous_timestamp = 0
        self.direction = 0
        self.final_timestamp = 0
        self.monitor_size = [0, 0]

        self.showing_predictions = False
        self.original_shape = self.model_agent._shape
        bounds = self.original_shape.bounds
        self.model_agent_size_metres = min(abs(bounds[3] - bounds[1]), abs(bounds[2] - bounds[0]))
        self.colour = Colours.BLUE
        self.unique_id = self.model_agent.unique_id

    def set_extremes(self, extremes):
        self.extremes = extremes

    # endregion

    # region pre visualisation Functions

    def create_sprites(self):
        centre_point = self.original_shape.centroid
        self.original_shape = shapely.affinity.translate(self.original_shape, -centre_point.x, -centre_point.y)

        # create the original, based on the scale, a base image that you can return to.
        self._create_original_image()
        # create first instance of the image that will actually be drawn
        self._create_image_instance()

    def _create_original_image(self):
        # recentre the agent shape

        self.shape = shapely.affinity.scale(self.original_shape, self.model_display_scale, self.model_display_scale)

        self.original_image = pygame.Surface((2 * self.model_agent_size_pixels, 2 * self.model_agent_size_pixels),
                                             pygame.SRCALPHA, 32)
        self.shape = shapely.affinity.translate(self.shape, self.model_agent_size_pixels, self.model_agent_size_pixels)
        self.polygon_coords = list(self.shape.exterior.coords)
        pygame.draw.polygon(self.original_image, self.colour, self.polygon_coords, 1)

    def _create_image_instance(self):
        self.image = pygame.Surface((2 * self.model_agent_size_pixels, 2 * self.model_agent_size_pixels),
                                    pygame.SRCALPHA, 32)
        pygame.draw.polygon(self.image, self.colour, self.polygon_coords, 1)
        self.image_w, self.image_h = self.image.get_size()

    def set_image_scale(self):
        pixels_per_metre_x = self.monitor_size[0] / (self.extremes["max_x"] - self.extremes["min_x"])
        pixels_per_metre_y = self.monitor_size[1] / (self.extremes["max_y"] - self.extremes["min_y"])

        size_of_agent_in_metres = self.model_agent_size_metres
        size_of_agent_in_pixels = int(size_of_agent_in_metres * pixels_per_metre_x)
        self.model_agent_size_pixels = size_of_agent_in_pixels
        self.model_display_scale = self.model_agent_size_pixels / self.model_agent_size_metres

    def setup(self, size):
        self.set_monitor_size(size)
        self.set_image_scale()
        self.standardise_positional_values()
        self.standardise_predictions()
        self.create_sprites()

        self.previous_timestamp = min(self.raw_predictions.keys())
        self.final_timestamp = max(self.raw_actual_trajectory.keys())

    def standardise_positional_values(self):
        self.standardised_actual_trajectory = self.standardise_trajectory(self.raw_actual_trajectory)
        self.standardised_planned_trajectory = self.standardise_trajectory(self.raw_planned_trajectory)

    def standardise_predictions(self):
        # there might be a better way of storing these? each agent has a prediction set which is a trajectory?
        # currently it's stored as {t_1: {id_1: waypoint_1, id_2:waypoint_2}, ... }
        self.standardised_predictions = self.raw_predictions.copy()

        for timestamp in self.raw_predictions:
            for prediction_step in self.raw_predictions[timestamp]:
                for agent in self.raw_predictions[timestamp][prediction_step]:
                    coord = self.standardise_waypoint(self.raw_predictions[timestamp][prediction_step][agent])
                    self.standardised_predictions[timestamp][prediction_step][agent] = coord

    def standardise_waypoint(self, coord):
        """ take each coord, standardise and then scale to monitor size"""
        coord = [(coord.position[0] - self.extremes["min_x"]) / (self.extremes["max_x"] - self.extremes["min_x"]),
                 (coord.position[1] - self.extremes["min_y"]) / (self.extremes["max_y"] - self.extremes["min_y"]),
                 coord.direction,
                 coord.t]
        return coord

    def standardise_trajectory(self, raw_trajectory=None):
        # this is where waypoints get converted into simple dicts of positions.
        standardised_trajectory = {}
        # each entry should be {t:[scaled_x, scaled_y, direction, time]}
        for i in raw_trajectory:
            coord = self.standardise_waypoint(raw_trajectory[i])
            standardised_trajectory[i] = coord
        return standardised_trajectory

    def set_planned_trajectory(self, trajectory):
        self.raw_planned_trajectory = trajectory

    def set_raw_trajectory(self, trajectory):
        self.raw_actual_trajectory = trajectory

    def set_raw_predictions(self, predictions):
        self.raw_predictions = predictions

    def set_monitor_size(self, size):
        self.monitor_size = size

    # endregion

    # region Runtime Functions
    def update_offset(self, offset):
        self.draw_offset = offset

    def update_scale(self, scale, sc):
        self.draw_scale = scale
        self.model_agent_size_pixels *= sc
        self.model_display_scale *= sc

        self._create_original_image()
        self._create_image_instance()

    def draw(self, timestamp, surface):
        ##### draw agent and trace points on surface#########
        if timestamp > self.final_timestamp:
            return
        if timestamp in self.standardised_actual_trajectory.keys():

            self.x = self.standardised_actual_trajectory[timestamp][0]
            self.y = self.standardised_actual_trajectory[timestamp][1]

            direction = self.standardised_actual_trajectory[timestamp][2]
            if abs(direction - self.direction) > 0.01:
                self.image = pygame.transform.rotate(self.original_image, -math.degrees(direction))
                self.image_w, self.image_h = self.image.get_size()
                self.direction = direction

            # this is to centre the agent image.
            image_x = self.x * self.draw_scale * self.monitor_size[0] \
                      - self.image_w / 2 + self.draw_offset[0]
            image_y = self.y * self.draw_scale * self.monitor_size[1] \
                      - self.image_h / 2 + self.draw_offset[1]
            surface.blit(self.image, (image_x, image_y))
            if self.trace.full():
                self.trace.get()

            self.trace.put((self.x, self.y))
            self._draw_trace(surface)

    def _draw_trace(self, surface):
        for t in self.trace.queue:
            self.rect = pygame.Rect(t[0] * self.draw_scale * self.monitor_size[0] + self.draw_offset[0],
                                    t[1] * self.draw_scale * self.monitor_size[1] + self.draw_offset[1],
                                    2, 2)
            pygame.draw.rect(surface, Colours.WHITE, self.rect)

    def _draw_predictions_coloured(self, surface, colour, timestamp):
        ### the predictions can go several timestamps ahead of the future.
        ### need to extract each timestamp, before plotting.

        for prediction_step in self.standardised_predictions[timestamp]:
            ## then there is another layer, where all the relevant agent predictions are stored
            for agent_id in self.standardised_predictions[timestamp][prediction_step]:
                if self.standardised_predictions[timestamp][prediction_step][agent_id][3] > timestamp:

                    coord = [self.standardised_predictions[timestamp][prediction_step][agent_id][0],
                             self.standardised_predictions[timestamp][prediction_step][agent_id][1]]

                    image_x = coord[0] * self.draw_scale * self.monitor_size[0]
                              # - self.image_w / 2 + self.draw_offset[0]
                    image_y = coord[1] * self.draw_scale * self.monitor_size[1]
                              # - self.image_h / 2 + self.draw_offset[1]

                    self.rect = pygame.Rect(image_x + self.draw_offset[0], image_y + self.draw_offset[1], 2, 2)
                    if agent_id == self.unique_id:
                        pygame.draw.rect(surface, Colours.PURPLE, self.rect)
                    else:
                        pygame.draw.rect(surface, colour, self.rect)

    def draw_predictions(self, timestamp, surface):
        """ draw the predictions for each other agent, defined at each timestep"""
        # check if there are any valid predictions, then draw them
        if timestamp > self.final_timestamp:
            return
        if timestamp in self.standardised_predictions.keys():
            self._draw_predictions_coloured(surface, Colours.RED, timestamp)
            self.previous_timestamp = timestamp
        elif self.previous_timestamp in self.standardised_predictions.keys():

            self._draw_predictions_coloured(surface, Colours.RED, self.previous_timestamp)

    def draw_planned_trajectory(self, timestamp, surface):
        ''' draw the agents planned motion'''

        for key in self.standardised_planned_trajectory.keys():
            if key > timestamp:
                coord = self.standardised_planned_trajectory[key]
                self.rect = pygame.Rect(coord[0] * self.draw_scale * self.monitor_size[0] + self.draw_offset[0],
                                        coord[1] * self.draw_scale * self.monitor_size[1] + self.draw_offset[1],
                                        2, 2)
                pygame.draw.rect(surface, Colours.PURPLE, self.rect)

    def toggle_highlighting(self):
        self.showing_predictions = not self.showing_predictions
        if self.showing_predictions:
            self.colour = Colours.ORANGE
        else:
            self.colour = Colours.BLUE

        # now overwrite the both base and drawn images with the correct colour
        pygame.draw.polygon(self.original_image, self.colour, self.polygon_coords, 1)
        self.image = pygame.transform.rotate(self.original_image, -math.degrees(self.direction))
        self.image_w, self.image_h = self.image.get_size()
    # endregion
