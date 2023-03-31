import os

project_path = os.path.dirname(os.path.realpath(__file__))
recording = False

simulation_fps = 40

input_file_path = f"{project_path}/input_data"
output_file_path = f"{project_path}/output_data"

input_image_file_path = f"{input_file_path}/assets/Images/"
input_video_file_path = f"{input_file_path}/assets/Videos/"
input_gis_file_path = f"{input_file_path}/assets/GIS/"
output_image_file_path = f"{output_file_path}/images/"
output_video_file_path = f"{output_file_path}/videos/"
temp_image_file_path = f"{output_image_file_path}/temp/"

cache_data_file_path = f"{input_file_path}/manual_cache/"

all_paths = [input_file_path, output_file_path, input_image_file_path, input_video_file_path, input_gis_file_path,
             output_image_file_path, output_video_file_path, temp_image_file_path, cache_data_file_path]

for path in all_paths:
    if not os.path.exists(path):
        os.mkdir(path)
