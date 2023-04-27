import os
from enum import Enum, auto


class Interface_Modes(Enum):
    Manual = auto()
    RL = auto()

class RL_Modes(Enum):
    Train = auto()
    Test = auto()

PROFILING=True
RECORDING = False
SIMULATION_FPS = 40

OBSTACLES_ON = False
OBSTACLE_NUMBER = 3
EGO_VEHICLE_SELF_DRIVE = False
EGO_VEHICLE_PERCEIVE = False
TRAFFIC_ON = False

INTERFACE_MODE = Interface_Modes.Manual
RL_MODE = RL_Modes.Train

VEHICLE_NUMBER = 20
PEDESTRIAN_NUMBER = 10 # TODO: haven't implemented this yet. look at the generate_traffic.py example if you want to

DISPLAY_ON = False
if INTERFACE_MODE == Interface_Modes.RL:
    if RL_MODE == RL_Modes.Train:
        DISPLAY_ON = False



project_path = os.path.dirname(os.path.realpath(__file__))
INPUT_FP = f"{project_path}/input_data"
OUTPUT_FP = f"{project_path}/output_data"

INPUT_IMAGE_FP = f"{INPUT_FP}/assets/Images"
INPUT_VIDEO_FP = f"{INPUT_FP}/assets/Videos"
INPUT_GIS_FP = f"{INPUT_FP}/assets/GIS"
OUTPUT_IMAGE_FP = f"{OUTPUT_FP}/images"
OUTPUT_VIDEO_FP = f"{OUTPUT_FP}/videos"
_temp_image_folder = f"{OUTPUT_IMAGE_FP}/temp"


MODEL_PATH = f"{OUTPUT_FP}/Models"
LOG_PATH = f"{OUTPUT_FP}/Logs"

all_paths = [INPUT_FP, OUTPUT_FP, INPUT_IMAGE_FP, INPUT_VIDEO_FP, INPUT_GIS_FP,
             OUTPUT_IMAGE_FP, OUTPUT_VIDEO_FP, _temp_image_folder]

for path in all_paths:
    if not os.path.exists(path):
        os.mkdir(path)
