import config
from visualisation.PygameVisualiser.PygameVisualisation import Visualisation
from visualisation.PygameVisualiser.World import World
from visualisation.PygameVisualiser.OSMData import get_edinburgh
import json
import os

def get_important_data(cached_filepath):
    road_network, optimal_route, background_img, background_img_extents = get_edinburgh()
    data_to_save={}
    for i, node_id in enumerate(optimal_route):
        x = road_network.nodes[node_id]['x']
        y = road_network.nodes[node_id]['y']
        data_to_save[i]=[x,y]
    data_to_save['background_img_extents']=background_img_extents
    out_file = open(cached_filepath, "w")
    json.dump(data_to_save, out_file, indent=6)
    out_file.close()

def main():
    data={}
    cached_filepath=config.cache_data_file_path+"edinburgh_nodes_and_extents.json"
    if not os.path.exists(cached_filepath):
        get_important_data(cached_filepath)

    with open(cached_filepath, 'r') as f:
        data = json.load(f)

    extents= data.pop('background_img_extents', None)
    pf = config.visualisation_padding_factor
    extents =[extents[0], extents[1]+pf, extents[2]-pf, extents[3]+pf]
    # world = World(road_network, optimal_route)
    world = World(override_waypoints=data, extents=extents)
    # v = Visualisation(world, background_img, background_img_extents)
    v=Visualisation(world)
    v.run()


if __name__ == "__main__":
    main()
