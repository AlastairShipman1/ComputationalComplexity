import numpy as np
import pygame
import shapely.geometry
from visualisation.VisualisationUtils import Colours
import config
from shapely.geometry import LineString


class Image(pygame.sprite.Sprite):
    def __init__(self, ego_vehicle=False):
        pygame.sprite.Sprite.__init__(self)
        image_path = config.input_image_file_path + '/blue_car_top_down.png'
        if ego_vehicle is None:
            image_path = config.input_image_file_path + '/red_car_top_down.png'

        self.image = pygame.image.load(image_path)
        self.image = pygame.transform.rotate(self.image, 180)

        scale = 0.2
        proposed_size = [self.image.get_width() * scale, self.image.get_height() * scale]
        self.image = pygame.transform.smoothscale(self.image, proposed_size)
        self.rect = self.image.get_rect()
        self.centre = self.rect.center


class Vehicle():

    # region Initialising functions
    def __init__(self, ego_vehicle=False):
        #### initialise position and direction ####
        self.v_long = 0
        self.v_lat = 0
        self.direction = 0
        self.acc_rate = 11
        self.friction_rate = 0.1
        self.turn_rate = 1
        self.max_v = 100
        self.original_image = Image(ego_vehicle)
        self.image = self.original_image.image
        self.offset = self.image.get_rect().center

        self.x = -self.offset[0] + 0
        self.y = -self.offset[1] + 150

    # endregion

    def move(self, dt=0):

        # move x by vlong*dt *cos(direction)
        # move y by vlong*dt*sin(direction)
        # reduce vlong by friction- which is bigger when faster
        # dt is in milliseconds.
        self.x += self.v_long * dt / 1000 * np.cos(np.deg2rad(self.direction))
        self.y -= self.v_long * dt / 1000 * np.sin(np.deg2rad(self.direction))
        friction_scaler = 3
        if abs(self.v_long) < 10:
            friction_scaler = 0.5
        self.v_long -= np.sign(self.v_long) * self.friction_rate * friction_scaler

        if abs(self.v_long) < 5:
            self.v_long = 0

    def update(self, dt):
        self.move(dt)

    def draw(self, surface):
        ##### draw agent on surface#########
        pos = (self.x - self.offset[0], self.y - self.offset[1])
        surface.blit(self.image, pos)

    def send_message(self, string):
        ...

    def rotate(self, angle):
        previous_image_center = self.image.get_rect().center
        rotated_image = pygame.transform.rotate(self.original_image.image, angle)
        rotated_image_center = rotated_image.get_rect().center
        offset = [rotated_image_center[0] - previous_image_center[0],
                  rotated_image_center[1] - previous_image_center[1]]
        self.image = rotated_image

        self.offset = [self.offset[0] + offset[0], self.offset[1] + offset[1]]


class EgoVehicle(Vehicle):
    def __init__(self, world):
        super().__init__(ego_vehicle=True)
        self.world = world
        self.rays = []

    def send_message(self, string):
        if string == "u":
            v_long = self.v_long + self.acc_rate
            if v_long > abs(self.max_v):
                v_long = self.max_v
            self.v_long = v_long

        if string == "d":
            v_long = self.v_long - self.acc_rate
            if abs(v_long) > self.max_v:
                v_long = -self.max_v
            self.v_long = v_long

        if string == "l":
            self.direction += self.turn_rate
            self.rotate(self.direction)

        if string == "r":
            self.direction -= self.turn_rate
            self.rotate(self.direction)

    def draw(self, surface):
        ##### draw agent on surface#########
        pos = (self.x - self.offset[0], self.y - self.offset[1])
        surface.blit(self.image, pos)

        self.draw_sensor_rays(surface)
        self.draw_sensor_area(surface)

    def draw_sensor_area(self, surface):
        pygame.draw.aalines(surface, Colours.BLACK, True, self.rays)
        covered_area = shapely.geometry.Polygon(self.rays)
        max_area = shapely.geometry.Point((self.x, self.y)).buffer(200)
        diff = max_area.difference(covered_area)
        if isinstance(diff, shapely.geometry.Polygon):
            diff = shapely.geometry.MultiPolygon([diff])
        for geom in diff.geoms:
            pygame.draw.polygon(surface, Colours.BLACK, list(geom.exterior.coords))

    def draw_sensor_rays(self, surface):
        ...
        #  this needs to send out rays from a position on the car, across 360deg
        #  then, if these hit an obstacle, they should stop at the point of that obstacle.
        #  then we need to fill in the unknown areas behind these rays.

        # start with circular rays
        self.rays = []
        depth = 200
        angle = self.direction
        number_rays = 100
        for i in range(number_rays):
            # create a line between the car and the max length
            # cycle through all obstacles
            target_x = self.x + np.cos(np.deg2rad(angle)) * depth
            target_y = self.y - np.sin(np.deg2rad(angle)) * depth
            l = LineString([(self.x, self.y), (target_x, target_y)])
            for obs in self.world.obstacles:
                # if the line goes through the obstacle, then find the nearest point on the obstacle
                # and store this
                if l.intersects(obs.boundary):
                    p = l.intersection(obs.boundary)

                    curr_dist = (self.x - target_x) ** 2 + (self.y - target_y) ** 2
                    if isinstance(p, shapely.geometry.Point):
                        p = shapely.geometry.MultiPoint([p])

                    for point in list(p.geoms):
                        # pygame.draw.circle(surface, Colours.BLUE, (point.x, point.y), 4)
                        temp_target_x, temp_target_y = point.x, point.y
                        temp_dist = (self.x - temp_target_x) ** 2 + (self.y - temp_target_y) ** 2
                        if temp_dist < curr_dist:
                            target_x = temp_target_x
                            target_y = temp_target_y
                            curr_dist = temp_dist

            self.rays.append([target_x, target_y])
            pygame.draw.aaline(surface, (255, 255, 0), (self.x, self.y), (target_x, target_y))

            angle -= 360 / number_rays

    '''
    # this is the c# code that does it=-
    int rayIdx = 1;

        for (int i = -halfVerticalRays; i < halfVerticalRays; i++)
        {
            // create a set of rays from the origin of this gameobject, 
            // shoot them out to the distance required.
            for (int ii = -halfHorizontalRays; ii < halfHorizontalRays; ii++)
            {
                /*
                there is a ray, sent out at an azimuth to the x-axis (psi, or ii*horizontalRes) and an inclination to the vertical (theta, or 90-i*verticalRes)
                to get the cartesian coordinates of the end point of that ray, we use a coordinate substitution:

                x=rcos(psi)sin(theta)
                y=rsin(psi)sin(theta)
                z=rcos(theta)
                */
                float inclination = Mathf.Deg2Rad * (90 - i * verticalRes) + forwardInclination;
                float azimuth = Mathf.Deg2Rad * ii * horizontalRes + forwardAzimuth;

                float maxTheoreticalRange = CalculateRange( i * verticalRes, ii * horizontalRes);
                float maxActualRange = maxTheoreticalRange * weatherRangeMultiplier;


                float dirX = Mathf.Cos(azimuth) * Mathf.Sin(inclination);
                float dirZ = Mathf.Sin(azimuth) * Mathf.Sin(inclination);
                float dirY = Mathf.Cos(inclination);
                Vector3 endPos = new Vector3(dirX, dirY, dirZ) * maxActualRange;

                RaycastHit hit;
                if (Physics.Raycast(transform.position, endPos, out hit, maxActualRange))
                {
                    // if it hits a collider, then stop it short, and record the length of the ray, the position of the endpoint
                    // store the end points of the ray (relative to the local frame)
                    endPos = hit.point - transform.position;
                }

                // store the vertex so that we can create a mesh out of it later.
                meshVertices[rayIdx] = endPos;
                rayIdx++;

                // also, draw a line showing what it looks like.
                //Debug.DrawLine(transform.position, endPos + transform.position, Color.blue, .05f);
            }
        }

    def cast_rays():
        start_angle = player_angle - HALF_FOV

        for ray in range(CASTED_RAYS):
            for depth in range(MAX_DEPTH):
                target_x = player_x - math.sin(start_angle) * depth
                target_y = player_y + math.cos(start_angle) * depth
                col = int(target_x / TILE_SIZE)
                row = int(target_y / TILE_SIZE)

                square = row * MAP_SIZE + col
                (target_y / TILE_SIZE) * MAP_SIZE + target_x / TILE_SIZE
                if MAP[square] == '#':
                    pygame.draw.rect(win, (0, 255, 0), (col * TILE_SIZE,
                                                        row * TILE_SIZE,
                                                        TILE_SIZE - 2,
                                                        TILE_SIZE - 2))
                    pygame.draw.line(win, (255, 255, 0), (player_x, player_y), (target_x, target_y))
                    color = 50 / (1 + depth * depth * 0.0001)

                    depth *= math.cos(player_angle - start_angle)

                    wall_height = 21000 / (depth + 0.0001)

                    if wall_height > SCREEN_HEIGHT: wall_height == SCREEN_HEIGHT

                    pygame.draw.rect(win, (color, color, color), (
                        SCREEN_HEIGHT + ray * SCALE, (SCREEN_HEIGHT / 2) - wall_height / 2, SCALE, wall_height))

                    break

            start_angle += STEP_ANGLE
'''
