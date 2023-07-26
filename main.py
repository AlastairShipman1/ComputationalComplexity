import pygame
from stable_baselines3 import PPO, A2C, SAC
import config
from RL.RLEnvironment import RLEnvironment
from model.Environment.OSMData import get_edinburgh
from visualisation.PygameVisualiser.PygameVisualisation import Visualisation
from model.Environment.World import World
from model.Agent import Control

def RL_analysis(world, visuals):
    try:
        # create the environment. this is the base unit for the reinforcement learning
        RLenv = RLEnvironment(world, visuals)
    except Exception as e:
        print(f"Can't make RL environment: {e}")
        return

    TOTAL_TIMESTEPS = 100000
    if config.RL_MODE == config.RL_Modes.Train:
        model = PPO('MlpPolicy', RLenv, verbose=1, tensorboard_log=config.LOG_PATH)
        for i in range(1500):
            model.learn(total_timesteps=TOTAL_TIMESTEPS, reset_num_timesteps=False, tb_log_name="PPO")
            model.save(f"{config.MODEL_PATH}/{TOTAL_TIMESTEPS * i}")
    else:
        # you should choose which model to load at this point
        model = PPO.load(f"{config.MODEL_PATH}/2200000", env=RLenv)
        # Game loop
        done = False
        obs = RLenv.reset()
        while not done:
            action = model.predict(obs)[0]
            obs, reward, done, info = RLenv.step(action)


def manual_mode(world, v= None):
    t = 0
    done = False
    pygame.init()
    control_obj = Control.PygameControlObject(world)

    while not done:
        t += 1
        done = control_obj.process_control()
        world.update(config.DELTA_TIME_MS) # milliseconds
        if config.DISPLAY_ON:
            v.update()
        if t == 100000:
            done = True
    if v is not None:
        v.quit()

def main():
    # road_network, optimal_route, background_img, background_img_extents = get_edinburgh()
    # road network and optimal route are used in the world
    # background img and extents are used in the visualisation
    world = World()
    v=None

    if config.DISPLAY_ON:
        v = Visualisation(world)
    if config.INTERFACE_MODE == config.Interface_Modes.Manual:
        manual_mode(world, v)
    try:
        if config.INTERFACE_MODE == config.Interface_Modes.RL:
            RL_analysis(world, v)
    except Exception as e:
        print(f"Error in Main: {e}")
    finally:
        if v is not None:
            v.quit()


if __name__ == "__main__":
    main()
