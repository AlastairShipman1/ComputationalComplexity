import shapely

from visualisation.PygameVisualiser.Vehicle import EgoVehicle
from shapely.geometry import Polygon

class World():
    def __init__(self):
        self.obstacles=[]
        polygon = Polygon([(100, 100), (200, 100), (200, 200), (100,200)])
        polygon1 = Polygon([(100, 300), (200, 300), (200, 400), (100,400)])

        polygon2 = Polygon([(300, 100), (400, 100), (400, 200), (300,200)])
        polygon3 = Polygon([(300, 300), (400, 300), (400, 400), (300,400)])

        polygon4 = Polygon([(500, 100), (600, 100), (600, 200), (500,200)])
        polygon5 = Polygon([(500, 300), (600, 300), (600, 400), (500,400)])

        polygon6 = Polygon([(700, 100), (800, 100), (800, 200), (700,200)])
        polygon7 = Polygon([(700, 300), (800, 300), (800, 400), (700,400)])

        polygon7 = shapely.geometry.Point((750,350)).buffer(50)



        self.obstacles=shapely.geometry.MultiPolygon([polygon, polygon1, polygon2, polygon3, polygon4, polygon5, polygon6, polygon7])
        self.ego_vehicle = EgoVehicle(self)
        self.pygame_agents = [self.ego_vehicle]

