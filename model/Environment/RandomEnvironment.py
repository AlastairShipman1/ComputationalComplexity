from src.Model.Agent.Agent import Agent
from src.Model.Environment.BaseEnvironment import BaseEnvironment


class RandomEnvironment (BaseEnvironment):

    def __init__(self, clock):

        super().__init__()
        self._clock=clock
        self.rng=self._clock.rng
        self._agents=[]
        self._fixed_obstacles=[]

        self.create_obstacle_list()
        self.points = []
        for i in range(100):
            for ii in range(100):
                self.points.append([i//10, ii//10])
                self.points.append([ii//10, i//10])

    def generate_start_point(self):
        return self._clock.rng.choice(self.points)

    def generate_route(self, min_route_length, max_route_length):
        route = {}
        agent_route_length = self.rng.integers(min_route_length, max_route_length, size=1, endpoint=True)[0]
        agent_route_points = self.rng.choice(self.points, size=agent_route_length, replace=False)
        for i in range(agent_route_length):
            route[i] = agent_route_points[i]
        return route


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
        ox, oy = [], []
        # a line from (-10, -10) to (59, -10)
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10)
        # a line from (60, -10) to (60, 59)
        for i in range(-10, 60):
            ox.append(60)
            oy.append(i)
        # a line from (-10, 60) to (60, 60)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60)
        # a line from (-10, -10) to (-10, 60)
        for i in range(-10, 61):
            ox.append(-10)
            oy.append(i)
        # a line from (20, -10) to (20, 39)
        for i in range(-10, 40):
            ox.append(20)
            oy.append(i)
        # a line from (40, 60) to (40, 20)
        for i in range(0, 40):
            ox.append(40)
            oy.append(60 - i)

        self.obstacles=[ox, oy]

    @property
    def fixed_obstacles(self):
        return self._fixed_obstacles
