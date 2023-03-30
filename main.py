from visualisation.PygameVisualiser.PygameVisualisation import Visualisation
from visualisation.PygameVisualiser.World import World
from visualisation.PygameVisualiser.OSMData import get_edinburgh

def main():
    road_network, optimal_route, background_img, background_img_extents = get_edinburgh()
    world = World(road_network, optimal_route)
    v = Visualisation(world, background_img, background_img_extents)
    v.run()


if __name__ == "__main__":
    main()
