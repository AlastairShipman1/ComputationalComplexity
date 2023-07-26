import os
from enum import Enum, auto

class Interface_Modes(Enum):
    Manual = auto()
    RL = auto()

class RL_Modes(Enum):
    Train = auto()
    Test = auto()

############################
## Simulation meta config ##
############################
PROFILING = False
RECORDING = False
INTERFACE_MODE = Interface_Modes.Manual
RL_MODE = RL_Modes.Train

DISPLAY_ON = True
if INTERFACE_MODE == Interface_Modes.RL:
    if RL_MODE == RL_Modes.Train:
        DISPLAY_ON = False

#############################
## World simulation config ##
#############################
DELTA_TIME_MS = 100
SIMULATION_FPS = int(1000/DELTA_TIME_MS)
VIS_PADDING_FACTOR=1000
OBSTACLES_ON = True
OBSTACLE_NUMBER = 3
EGO_VEHICLE_SELF_DRIVE = False
EGO_VEHICLE_PERCEIVE = True
TRAFFIC_ON = False
VEHICLE_NUMBER = 20
PEDESTRIAN_NUMBER = 10 # TODO: haven't implemented this yet. look at the generate_traffic.py example if you want to

#######################################
## Ensure that directories are valid ##
#######################################
project_path = os.path.dirname(os.path.realpath(__file__))
INPUT_FP = f"{project_path}/input_data"
OUTPUT_FP = f"{project_path}/output_data"

INPUT_IMAGE_FP = f"{INPUT_FP}/assets/Images"
INPUT_VIDEO_FP = f"{INPUT_FP}/assets/Videos"
INPUT_GIS_FP = f"{INPUT_FP}/assets/GIS"
OUTPUT_IMAGE_FP = f"{OUTPUT_FP}/images"
OUTPUT_VIDEO_FP = f"{OUTPUT_FP}/videos"
OUTPUT_RL_FP = f"{OUTPUT_FP}/RL"
_temp_image_folder = f"{OUTPUT_IMAGE_FP}/temp"

RL_MODEL_PATH = f"{OUTPUT_RL_FP}/Models"
RL_LOG_PATH = f"{OUTPUT_RL_FP}/Logs"

all_paths = [INPUT_FP, OUTPUT_FP, INPUT_IMAGE_FP, INPUT_VIDEO_FP, INPUT_GIS_FP,
             OUTPUT_IMAGE_FP, OUTPUT_VIDEO_FP, RL_LOG_PATH, RL_MODEL_PATH, _temp_image_folder]

for path in all_paths:
    if not os.path.exists(path):
        os.makedirs(path)