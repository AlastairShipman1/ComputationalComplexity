import cv2
from stable_baselines3.common.env_checker import check_env

from Environment import CarlaEnvironment
from OwnWork.SimulationEnv.CarlaConnector import CarlaConnection

connection = CarlaConnection()
client, world = connection.setup()
env = CarlaEnvironment(client, world)

# It will check your custom environment and output additional warnings if needed
check_env(env)

episodes = 5

for episode in range(episodes):
    done = False
    obs = env.reset()
    i = 0
    while not done:
        random_action = env.action_space.sample()
        obs, reward, done, info = env.step(random_action)
        # for some reason this can't be done in the render object?
        k = cv2.waitKey(1)
        # you can do this in pygame as well/instead if you want
        i += 1
        if i > 1000 or k == 27:
            done = True

env.quit()
