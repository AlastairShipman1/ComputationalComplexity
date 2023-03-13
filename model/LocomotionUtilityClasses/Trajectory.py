
import numpy as np


class Trajectory:
    def __init__(self, waypoints:list(Waypoint)):
        if not (isinstance(waypoints, list) and all(isinstance(waypoints, Waypoint))):
            raise TypeError("waypoints should be a list of Waypoints")

        self.waypoints=waypoints

    def trajectory_duration(self):
        return self.waypoints[-1].t-self.waypoints[0].t

    def trajectory_total_acceleration(self):
        acc_sum=0
        for i in range(1, len(self.waypoints)):
            v_diff= np.sqrt(self.waypoints[i].v_x**2+self.waypoints[i].v_y**2) \
                    - np.sqrt(self.waypoints[i-1].v_x**2+self.waypoints[i-1].v_y**2)
            t_diff= self.waypoints[i].t-self.waypoints[i-1].t

            acc_sum+= v_diff/t_diff
        return acc_sum


