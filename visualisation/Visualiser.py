import numpy as np
from model.Agent import Agent
from visualisation.PygameVisualiser.PygameAgent import PygameAgent
from visualisation.PygameVisualiser import PygameVisualisation
from scenarios.Scenario import Scenario
from visualisation import OSMTiles
import config


class Visualiser:
    def __init__(self, scenario: Scenario):
        self.agents = scenario.agents
        self.clock = scenario.clock
        self.visualise(self.agents, self.clock)

    def visualise(self, agents, clock):
        if not (isinstance(agents, list) and all(isinstance(agent, Agent.Agent) for agent in agents)):
            raise TypeError("Type not supported. Need to enter a list of agents into the visualisation")

        """ used as the incoming function to visualise. should this be a full class in itself?"""
        #### create the sprites here.
        agent_sprites = []
        trajectories = {}

        for agent in agents:
            trajectories[agent.unique_id] = agent.past_trajectory
        # it might be useful getting extreme values from an environment instead? but then we might still have
        # scaling problems
        extremes = self.find_extreme_values_in_trajectories(trajectories)

        for agent in agents:
            x = PygameAgent(agent)
            x.set_extremes(extremes)
            agent_sprites.append(x)

        if not config.visualisation_metrics["use_existing_bg"]:
            OSMTiles.generate_background(extremes, (51.501748, -0.223882))

        visualisation = PygameVisualisation.Visualisation(agent_sprites, clock)
        agent_sprites[0].toggle_highlighting()
        # agent_sprites[1].toggle_highlighting()

        visualisation.run()

    def find_extreme_values_in_trajectories(self, trajectories):
        max_x, max_y, max_t = -np.inf, -np.inf, -np.inf
        min_x, min_y, min_t = np.inf, np.inf, np.inf

        for traj in trajectories.keys():
            for i in trajectories[traj]:
                max_x = trajectories[traj][i].position[0] if trajectories[traj][i].position[0] > max_x else max_x
                max_y = trajectories[traj][i].position[1] if trajectories[traj][i].position[1] > max_y else max_y
                min_x = trajectories[traj][i].position[0] if trajectories[traj][i].position[0] < min_x else min_x
                min_y = trajectories[traj][i].position[1] if trajectories[traj][i].position[1] < min_y else min_y
            temp_min_t = min(trajectories[traj].keys())
            temp_max_t = max(trajectories[traj].keys())
            min_t = min(temp_min_t, min_t)
            max_t = max(temp_max_t, max_t)

        # maybe add a 10% buffer?
        max_x += 10
        min_x -= 10
        max_y += 10
        min_y -= 10
        return {"max_x": max_x, "min_x": min_x, "max_y": max_y, "min_y": min_y, "max_t": max_t, "min_t": min_t}
