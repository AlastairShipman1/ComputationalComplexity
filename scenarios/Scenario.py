from model.Clock import Clock
from model.Environment.RandomEnvironment import RandomEnvironment
from model.Environment.CorridorEnvironment import CorridorEnvironment
from model.Agent.Human import Human
from tqdm import tqdm

class Scenario():
    def __init__(self):
        self.max_simulation_time = None
        self.agents = None
        self.clock = None
        ...

    def run(self):
        # this is currently blocking. should maybe do this async?. w
        # will need to refactor this if you want to put in any controller inputs (and have interaction)
        # possibly after moving from python?
        for _ in tqdm(self._run()):
            ...
        return


    def _run(self):
        while True:
            self.clock.tick()
            condition_to_break_simulation = True
            for agent in self.agents:
                if not agent.completed:
                    condition_to_break_simulation = False

            # don't run for more than Xs of simtime
            if self.clock.get_time() > self.max_simulation_time or condition_to_break_simulation:
                break
            yield


class RandomScenario(Scenario):
    def __init__(self):
        super().__init__()
        # random scenario
        self.num_agents = 10
        self.max_route_length = 5
        self.min_route_length = 3
        self.max_simulation_time=50
        self.clock = Clock()
        self.environment = RandomEnvironment(self.clock)
        self.agents = []
        self.generate_agents_and_routes()

    def generate_agents_and_routes(self):
        for i in range(self.num_agents):
            x = Human(self.clock, i + 1)
            self.agents.append(x)

            # this doesn't seem like a good way of doing it?
            x.environment = self.environment
            self.environment.add_agent(x)
            starting_point = self.environment.generate_start_point()
            route = self.environment.generate_route(self.min_route_length, self.max_route_length)

            x.set_origin(starting_point)
            x.set_route(route)





class BasicCorridorScenario(Scenario):

    def __init__(self):
        super().__init__()
        self.num_agents = 2
        self.max_simulation_time = 50
        self.clock = Clock()
        self.environment = CorridorEnvironment(self.clock)
        self.agents = []
        self.generate_agents_and_routes()

    def generate_agents_and_routes(self):
        for i in range(self.num_agents):
            x = Human(self.clock, i + 1)
            self.agents.append(x)

            # this doesn't seem like a good way of doing it?
            x.environment = self.environment
            self.environment.add_agent(x)

            # this needs to be done AFTER setting the environment. change this?
            starting_point = self.environment.generate_start_point()
            route = self.environment.generate_route()

            x.set_origin(starting_point)
            x.set_route(route)



class ExitRoomScenario(Scenario):
    def __init__(self):
        super().__init__()
        self.num_agents=10
        self.max_simulation_time = 50
        self.clock = Clock()
        self.environment = RandomEnvironment(self.clock)
        self.agents = []
        self.generate_agents_and_routes()

    def generate_agents_and_routes(self):
        ...


