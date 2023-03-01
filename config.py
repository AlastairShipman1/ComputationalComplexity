import os


project_path = os.path.dirname(os.path.realpath(__file__))
recording = True

simulation_fps = 40

input_file_path = f"{project_path}/input_data"
output_file_path = f"{project_path}/output_data"

input_image_file_path= f"{input_file_path}/assets/Images/"
input_video_file_path= f"{input_file_path}/assets/Videos/"
output_image_file_path = f"{output_file_path}/images"
output_video_file_path = f"{output_file_path}/videos"
temp_image_folder = f"{output_image_file_path}/temp"


if not os.path.exists(input_file_path):
    os.mkdir(input_file_path)
if not os.path.exists(output_file_path):
    os.mkdir(output_file_path)
if not os.path.exists(output_image_file_path):
    os.mkdir(output_image_file_path)
if not os.path.exists(output_video_file_path):
    os.mkdir(output_video_file_path)
if not os.path.exists(temp_image_folder):
    os.mkdir(temp_image_folder)