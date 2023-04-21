from visualisation.PygameVisualiser.PygameVisualisation import Visualisation
from visualisation.PygameVisualiser.World import World
from visualisation.PygameVisualiser.OSMData import get_edinburgh

#
# def check_esc():
#     # you can do this in pygame as well/instead if you want
#     done = False
#     k = cv2.waitKey(1)
#     if k == 27:  # wait for esckey to exit
#         done = True
#     return done

def main():
    road_network, optimal_route, background_img, background_img_extents = None, None, None, None
    # road_network, optimal_route, background_img, background_img_extents = get_edinburgh()

    world = World(road_network, optimal_route)
    v = Visualisation(world, background_img, background_img_extents)
    v.run()

#
# def RL_analysis(env):
#     TOTAL_TIMESTEPS = 20_000
#     if config.RL_ANALYSIS_MODE == config.RL_Analysis_Mode.Training:
#         env.apply_settings(False)
#         model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=config.LOG_PATH)
#         for i in range(150):
#             model.learn(total_timesteps=TOTAL_TIMESTEPS, reset_num_timesteps=False, tb_log_name="PPO")
#             model.save(f"{config.MODEL_PATH}/{TOTAL_TIMESTEPS * i}")
#     else:
#         env.apply_settings(True)
#         # you should choose which model to load at this point
#         model = PPO.load(f"{config.MODEL_PATH}/2500000", env=env)
#         # Game loop
#         done = False
#         obs = env.reset()
#         while not done:
#             action = model.predict(obs)[0]
#             obs, reward, done, info = env.step(action)
#             done = check_esc()
#
#
# def manual_control(env):
#     done = False
#     env.apply_settings(True)
#     env.reset()
#
#     while not done:
#         env.step()
#         done = check_esc()


# def mai3n():
#     env = None
#
#     # cover the whole thing in a try/finally clause, to make sure that
#     # any actors and sensors are destroyed in the case of a script error
#     try:
#         # Connect to the client and retrieve the world object
#         connection = CarlaConnection()
#         client, world = connection.setup()
#
#         # create the environment. this is the base unit for the reinforcement learning
#         env = CarlaEnvironment(client, world)
#
#         if config.INTERFACE_MODE == config.Interface_Mode.RL:
#             RL_analysis(env)
#         elif config.INTERFACE_MODE == config.Interface_Mode.Manual:
#             manual_control(env)
#
#     finally:
#         if env is not None:
#             env.quit()


if __name__ == "__main__":
    main()
