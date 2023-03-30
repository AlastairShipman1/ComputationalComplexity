from model.LocomotionUtilityClasses.Waypoint import Waypoint
import numpy as np


class Trajectory:
    def __init__(self, waypoints:list(Waypoint)):
        if not (isinstance(waypoints, list) and all(isinstance(waypoints, Waypoint))):
            raise TypeError("waypoints should be a list of Waypoints")

        self.waypoints=waypoints



