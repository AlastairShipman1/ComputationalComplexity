import carla
import numpy as np
import pygame
import keyboard


class ControlObject:
    def __init__(self, env):
        # Conrol parameters to store the control state
        self.env = env
        self._vehicle = env.ego_vehicle
        self._steer = 0
        self._throttle = False
        self._brake = False
        self._steer = None
        self._steer_cache = 0
        # A carla.VehicleControl object is needed to alter the
        # vehicle's control state
        self._control = carla.VehicleControl()

    def process_control(self):
        # Process the current control state, change the control parameter
        # if the key remains pressed

        if self._throttle:
            self._control.throttle = min(self._control.throttle + 0.04, 1)
            self._control.gear = 1
            self._control.brake = False
        elif not self._brake:
            self._control.throttle = 0.0

        if self._brake:
            # If the down arrow is held down when the car is stationary, switch to reverse
            if self._vehicle.get_velocity().length() < 0.01 and not self._control.reverse:
                self._control.brake = 0.0
                self._control.gear = 1
                self._control.reverse = True
                self._control.throttle = min(self._control.throttle + 0.4, 1)
            elif self._control.reverse:
                self._control.throttle = min(self._control.throttle + 0.4, 1)
            else:
                self._control.throttle = 0.0
                self._control.brake = 1# min(self._control.brake + 0.4, 1)
        else:
            self._control.brake = 0.0

        if self._steer is not None:
            self._steer_cache += np.sign(self._steer) * 0.03
        else:
            self._steer_cache *= 0.2
            if 0.01 > self._steer_cache > -0.01:
                self._steer_cache = 0.0
        self._control.steer = round(self._steer_cache, 1)

        # Ápply the control parameters to the ego vehicle
        self._vehicle.apply_control(self._control)


# Control object to manage vehicle controls
class PygameControlObject(ControlObject):
    def __init__(self, env):
        super().__init__(env)

    # Check for key press events in the PyGame window
    # and define the control state

    def process_control(self):
        for event in pygame.event.get():
            self.parse_control(event)
        super().process_control()

    def parse_control(self, event):
        if event.type == pygame.QUIT:
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                self._vehicle.set_autopilot(False)
            if event.key == pygame.K_UP:
                self._throttle = True
            if event.key == pygame.K_DOWN:
                self._brake = True
            if event.key == pygame.K_RIGHT:
                self._steer = 1
            if event.key == pygame.K_LEFT:
                self._steer = -1
            elif event.key == pygame.K_ESCAPE:
                return True
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                self._throttle = False
            if event.key == pygame.K_DOWN:
                self._brake = False
                self._control.reverse = False
            if event.key == pygame.K_RIGHT:
                self._steer = None
            if event.key == pygame.K_LEFT:
                self._steer = None
        return False


class KeyboardControlObject(ControlObject):
    def __init__(self, env):
        super().__init__(env)
        keyboard.on_press_key("up", lambda key: self.parse_control(key))
        keyboard.on_release_key("up", lambda key: self.parse_control(key))
        keyboard.on_press_key("down", lambda key: self.parse_control(key))
        keyboard.on_release_key("down", lambda key: self.parse_control(key))

        keyboard.on_press_key("left", lambda key: self.parse_control(key))
        keyboard.on_release_key("left", lambda key: self.parse_control(key))
        keyboard.on_press_key("right", lambda key: self.parse_control(key))
        keyboard.on_release_key("right", lambda key: self.parse_control(key))

    def parse_control(self, key):

        if key.event_type == keyboard.KEY_DOWN:
            if key.name == 'up':
                self._throttle = True
            if key.name == 'down':
                self._brake = True
            if key.name == 'left':
                self._steer = -1
            if key.name == 'right':
                self._steer = 1
        if key.event_type == keyboard.KEY_UP:
            if key.name == 'up':
                self._throttle = False
            if key.name == 'down':
                self._brake = False
                self._control.reverse = False
            if key.name == 'left':
                self._steer = None
            if key.name == 'right':
                self._steer = None


class RLControlObject(ControlObject):
    def __init__(self, env):
        super().__init__(env)

    def parse_control(self, action):
        throttle = action[0]
        steer = action[1]

        self._throttle = False
        self._steer = None
        self._brake = False

        if throttle == 2:
            self._throttle = True
            if self._control.reverse:
                self._control.reverse = False
        if throttle == 0:
            self._brake = True
        if steer != 1:
            self._steer = steer - 1
