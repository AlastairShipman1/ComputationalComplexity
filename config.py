import os

image_file_path="./Assets/Images/"
video_file_path="./Assets/Videos"

project_path = os.path.dirname(os.path.realpath(__file__))


input_file_path = f"{project_path}/input_data"
if not os.path.exists(input_file_path):
    os.mkdir(input_file_path)

example_traj_data_file_path= f"{project_path}/input_data/example_trajectories"
if not os.path.exists(example_traj_data_file_path):
    os.mkdir(example_traj_data_file_path)

output_file_path = f"{project_path}/output_data"
if not os.path.exists(output_file_path):
    os.mkdir(output_file_path)

output_image_file_path = f"{output_file_path}/images"
if not os.path.exists(output_image_file_path):
    os.mkdir(output_image_file_path)

output_video_file_path = f"{output_file_path}/videos"
if not os.path.exists(output_video_file_path):
    os.mkdir(output_video_file_path)

temp_image_folder = f"{output_image_file_path}/temp"
if not os.path.exists(temp_image_folder):
    os.mkdir(temp_image_folder)

import scenarios.Scenario as Scenario

#scenario=Scenario.RandomScenario()
scenario=Scenario.BasicCorridorScenario()
#
profiling=False

visualisation_metrics={"visualising": False,
                       "screen_prop": 0.8,
                       "figure_padding": 0,
                       "use_existing_bg": True,
                       "recording":False
                       }

image_file_path="./Assets/Images/"
video_file_path="./Assets/Videos"

bubble_generation_visualisation=True
