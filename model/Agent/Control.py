import numpy as np
import pygame
import keyboard


# Control object to manage vehicle controls
class PygameControlObject:
    def __init__(self, env):
        self.env=env

    # Check for key press events in the PyGame window
    # and define the control state

    def process_control(self):
        keys = pygame.key.get_pressed()  # checking pressed keys

        if keys[pygame.K_UP]:
            self.env.ego_vehicle.send_message("u")
        if keys[pygame.K_DOWN]:
            self.env.ego_vehicle.send_message("d")
        if keys[pygame.K_LEFT]:
            self.env.ego_vehicle.send_message("l")
        if keys[pygame.K_RIGHT]:
            self.env.ego_vehicle.send_message("r")


class KeyboardControlObject:
    def __init__(self, env):
        self.env=env
        keyboard.on_press_key("up", lambda key: self.parse_control(key))
        # keyboard.on_release_key("up", lambda key: self.parse_control(key))
        keyboard.on_press_key("down", lambda key: self.parse_control(key))
        # keyboard.on_release_key("down", lambda key: self.parse_control(key))

        keyboard.on_press_key("left", lambda key: self.parse_control(key))
        # keyboard.on_release_key("left", lambda key: self.parse_control(key))
        keyboard.on_press_key("right", lambda key: self.parse_control(key))
        # keyboard.on_release_key("right", lambda key: self.parse_control(key))

    def parse_control(self, key):
        if key.event_type == keyboard.KEY_DOWN:
            if key.name == 'up':
                self.env.ego_vehicle.send_message('u')
            if key.name == 'down':
                self.env.ego_vehicle.send_message('d')
            if key.name == 'left':
                self.env.ego_vehicle.send_message('l')
            if key.name == 'right':
                self.env.ego_vehicle.send_message('r')



class RLControlObject:
    def __init__(self, env):
        self.env=env

    def parse_control(self, action):
        throttle = action[0]
        steer = action[1]

        self._throttle = False
        self._steer = None
        self._brake = False

        if throttle == 2:
            self.env.ego_vehicle.send_message('u')
        if throttle == 0:
            self.env.ego_vehicle.send_message('d')
        if steer==0:
            self.env.ego_vehicle.send_message('l')
        if steer==2:
            self.env.ego_vehicle.send_message('r')