from abc import ABC, abstractmethod

import numpy as np


class AbstractWaypoint(ABC):

    '''
    just in case more waypoint types are needed in future
    e.g. differentiating between humans and cars.
    '''

    def __init__(self):
        ''' initialiser'''
    #
    # @abstractmethod
    # def assess_deviation(self, agent):
    #     """
    #     need to be able to assess how far the agent is from the waypoint
    #     this can be in time, velocity, or position
    #     """

class Waypoint(AbstractWaypoint):
    '''
        basic waypoint type
        this class defines the point that an agent should get to,
        at a time they should be there, and the velocity that they should have when they are
    '''
    def __init__(self, x, y, t, v_x, v_y, theta):
        self.position = [x, y]
        self.t = t
        self.velocity = [v_x, v_y]
        self.direction = theta

    def assess_deviation(self, agent):
        ''' check deviation'''
        pos_dev = self.assess_deviation_position(agent)
        rot_dev = self.assess_deviation_rotation(agent)
        vel_dev = self.assess_deviation_velocity(agent)
        deviation = {
            "position": pos_dev,
            "rotation": rot_dev,
            "velocity": vel_dev
        }

        return deviation

    def assess_deviation_rotation(self, agent):
        return self.direction- np.arctan2(agent.velocity[0], agent.velocity[1])

    def assess_deviation_position(self, agent):
        dist_squared=[(agent.position[0]-self.position[0]), (agent.position[1]-self.position[1])]
        return dist_squared


    def assess_deviation_velocity(self, agent):
        vel_dev= [(agent.velocity[0]-self.velocity[0]),(agent.velocity[1]-self.velocity[1])]
        return vel_dev


