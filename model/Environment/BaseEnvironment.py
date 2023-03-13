from abc import ABC, abstractmethod

class BaseEnvironment(ABC):
    """
    define a geometry, which can also be used to import data from, for example, osm/cadfiles/.

    define start points,
    then define the routes to get from point a to point b.
    readonly access to all agents, and clock,

    define the environment. this will be a set of non-overlapping polygons which define the walkable
    area, and associated characteristics (like road, pavement, walkway, etc). possibly formed by lanelets?
    the environment will provide limits on where the agent can move
    """

    def __init__(self):
        ...

    @property
    @abstractmethod
    def agents(self):
        ...

    @property
    @abstractmethod
    def fixed_obstacles(self):
        ...

    @property
    @abstractmethod
    def geometry(self):
        ...

    @abstractmethod
    def add_agent(self, agent):
        ...

    @abstractmethod
    def add_agents(self, agents):
        ...

    @abstractmethod
    def assign_environment_to_agents(self):
        ...
    @abstractmethod
    def generate_start_point(self):
        ...
    @abstractmethod
    def generate_route(self):
        ...

