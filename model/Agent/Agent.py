from model import Clock

from abc import ABC, abstractmethod

class Agent(ABC):

    def __init__(self, clock:Clock, uuid):
        self._unique_id= uuid
        self._clock = clock

    @property
    @abstractmethod
    def starting_tick(self):
        """  access the starting tick """

    @property
    @starting_tick.setter
    def starting_tick(self, starting_tick):
        """ set the starting tick """

    @abstractmethod
    def post_tick(self):
        """ each agent needs to be accessible by the clock """

    @abstractmethod
    def pre_tick(self):
        """ each agent needs to be accessible by the clock """

    @abstractmethod
    def tick(self):
        """ each agent needs to be accessible by the clock """

    @property
    @abstractmethod
    def environment(self):
        """ gets the environment"""

    @environment.setter
    @abstractmethod
    def environment(self, env):
        """sets the environment"""

    @property
    @abstractmethod
    def position(self):
        """ gets the position of the agent"""

    @property
    @abstractmethod
    def direction(self):
        """ gets the direction of the agent"""


    @property
    @abstractmethod
    def velocity(self):
        """ gets the velocity of the agent"""

    @property
    @abstractmethod
    def unique_id(self):
        """ gets the id of the agent"""

    @property
    @abstractmethod
    def clock(self):
        """ gets the clock of the agent"""


    @property
    @abstractmethod
    def shape(self):
        """ defines the shape of the agent, returns a geometry"""

    #### anything else?