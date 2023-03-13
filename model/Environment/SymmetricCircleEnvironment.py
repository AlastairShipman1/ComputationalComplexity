from src.Model.Agent.Agent import Agent
from src.Model.Environment.BaseEnvironment import BaseEnvironment

class SymmetricCircleEnvironment (BaseEnvironment):

    def __init__(self, clock):
        super().__init__()
        self._clock=clock
        self._agents=[]
        self.points = []
        self.length=5
        self._fixed_obstacles=[]

    def generate_start_point(self):
        # alternate between ends of the corridor
        i=len(self._agents)%2
        #[0,0] to [1,1] and [1,0] to [0,1]
        starting_point = [(i)*5 * self.length, (0) * self.length+i]
        return starting_point


    def generate_route(self):
        # alternate between ends of the corridor
        i = len(self._agents) % 2
        route = {
            0: [(1-i)*5 * self.length, (0)*self.length+0.25*i],
           # 1: [(2) * self.length, (2-i)*self.length],
           # 2: [(1.5) * self.length, (i+1) * self.length]
        }
        return route

    @property
    def fixed_obstacles(self):
        return self._fixed_obstacles

    #region Properties
    @property
    def geometry(self):
        ...

    @property
    def agents(self):
        return self._agents


    def add_agents(self, agents):
        if not (isinstance(agents, list) and all(isinstance(agent, Agent) for agent in agents)):
            raise TypeError("Type not supported. Need to enter a list of agents into the environment")
        self._agents=agents

    def add_agent(self, agent: Agent):
        self._agents.append(agent)

    def assign_environment_to_agents(self):
        for agent in self._agents:
            agent.environment = self

    #endregion

    def create_obstacle_list(self):
        # list of obstacles
        #these should be shapely polygons.
        #
        # ox, oy = [], []
        # # a line from (-10, -10) to (59, -10)
        # for i in range(-10, 60):
        #     ox.append(i)
        #     oy.append(-10)
        # # a line from (60, -10) to (60, 59)
        # for i in range(-10, 60):
        #     ox.append(60)
        #     oy.append(i)
        # # a line from (-10, 60) to (60, 60)
        # for i in range(-10, 61):
        #     ox.append(i)
        #     oy.append(60)
        # # a line from (-10, -10) to (-10, 60)
        # for i in range(-10, 61):
        #     ox.append(-10)
        #     oy.append(i)
        # # a line from (20, -10) to (20, 39)
        # for i in range(-10, 40):
        #     ox.append(20)
        #     oy.append(i)
        # # a line from (40, 60) to (40, 20)
        # for i in range(0, 40):
        #     ox.append(40)
        #     oy.append(60 - i)

        self._fixed_obstacles=[]
