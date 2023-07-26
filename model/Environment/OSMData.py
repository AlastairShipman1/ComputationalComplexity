import json

import numpy as np
import osmnx as ox
import networkx as nx
import config
import os
import geopandas as gpd
import contextily as cx

from pyproj import Proj, transform
import rasterio
from rasterio.plot import show as rioshow
import matplotlib.pyplot as plt

from visualisation.PygameVisualiser import OSMTiles


def main():
    get_edinburgh()


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

def get_edinburgh():
    edinburgh_graph_filepath = config.INPUT_GIS_FP + "Edinburgh.graphml"
    edinburgh_nodes_filepath = config.INPUT_GIS_FP + "Edinburgh_nodes.gpkg"
    edinburgh_streets_filepath = config.INPUT_GIS_FP + "Edinburgh_streets.gpkg"
    edinburgh_geometries_filepath = config.INPUT_GIS_FP + "Edinburgh_geometries.gpkg"
    all_filepaths = [edinburgh_graph_filepath, edinburgh_nodes_filepath, edinburgh_streets_filepath,
                     edinburgh_geometries_filepath]

    if not all([os.path.isfile(i) for i in all_filepaths]):
        place = "Coventry, UK"
        edi_graph = ox.graph_from_place(place, network_type='drive')
        ox.save_graphml(edi_graph, edinburgh_graph_filepath)
        edi_nodes, edi_streets = ox.graph_to_gdfs(edi_graph)
        # edi_streets.to_file(edinburgh_streets_filepath, driver="GPKG")
        # edi_nodes.to_file(edinburgh_nodes_filepath, driver="GPKG")

        # tags = {'building': True, 'highway': True}
        # edi_geoms = ox.geometries_from_place(place, tags={"highway": True})
        # edi_geoms = edi_geoms.apply(lambda c: c.astype(str) if c.name != "geometry" else c, axis=0)
        # edi_geoms.to_file(edinburgh_geometries_filepath, driver="GPKG")

    else:
        edi_graph = ox.load_graphml(edinburgh_graph_filepath)
        # edi_nodes = gpd.read_file(edinburgh_nodes_filepath)
        # edi_streets = gpd.read_file(edinburgh_streets_filepath)
        # edi_geoms = gpd.read_file(edinburgh_geometries_filepath)

    orig_node = list(edi_graph)[0]
    dest_node = list(edi_graph)[-1]

    edi_graph = ox.project_graph(edi_graph, to_crs='epsg:3857')
    route = ox.shortest_path(edi_graph, orig_node, dest_node, weight="length")
    n = edi_graph.nodes[dest_node]['y']
    s = edi_graph.nodes[orig_node]['y']
    e = edi_graph.nodes[dest_node]['x']
    w = edi_graph.nodes[orig_node]['x']

    # fig, ax = plt.subplots()
    # back_img, extents = cx.bounds2img(w, s, e, n, source=cx.providers.CartoDB.Voyager)
    # ax.imshow(back_img, extent=extents)
    ox.plot_graph_route(edi_graph, route, route_color="y", route_linewidth=6, node_size=0)
    # edi_graph.plot(ax=ax) # this will work for a geometries gdf, which you should do next.
    # plt.show()

    return edi_graph, route#, back_img, extents


# def maibn():
#     data_url = "https://ndownloader.figshare.com/files/20232174"
#     db = gpd.read_file(data_url)
#     ax = db.plot(color="red", figsize=(9, 9))
#     cx.add_basemap(ax, crs=db.crs.to_string())
#     zaragoza = db.query("city_id == 'ci122'")
#     ax = zaragoza.plot(facecolor="none",
#                        edgecolor="red",
#                        linewidth=2
#                       )
#     cx.add_basemap(ax,
#                    crs=zaragoza.crs.to_string(),
#                    source=cx.providers.CartoDB.Voyager
#                   )
#     plt.show()


if __name__ == "__main__":
    main()

    #     W, S, E, N = (
    #         3.70106218566894531,
    #         50.998912458110244,
    #         3.70947485351562,ww
    #         51.00103994019806845
    #     )
    #     # N=55.955
    #     # S=55.951
    #     # E=-3.189
    #     # W=-3.187
    #     # G=ox.geometries_from_bbox(N, S, E, W, tags)
    #     # # G=ox.geometries_from_point((55.9535,-3.1883 ), dist=800, tags=tags)
    #     # fig, ax = plt.subplots()
    #     # max_zoom=0
    #     # max_zoom_provider = cx.providers.CartoDB.Voyager
    #     # providers=cx.providers.flatten()
    #     # for prov in providers:
    #     #     retval = cx.tile._validate_zoom(28, providers[prov], auto=True)
    #     #     if retval>max_zoom:
    #     #         retval = max_zoom
    #     #         # max_zoom_provider=providers[prov]
    #
    #     # print(retval)
    #     # G.plot(ax=ax)
    # cx.add_basemap(ax, crs=G.crs.to_string(),source=max_zoom_provider)
    # plt.show()
    #     ghent_img, ghent_ext = cx.bounds2img(W,S,E,N,ll=True, zoom=19,
    #                                          source=cx.providers.CartoDB.Voyager
    #                                          )
    #     print(ghent_ext)
    #     plt.imshow(ghent_img, extent=ghent_ext)
    #     # G.plot(ax=ax)
    #     plt.show()
    #     # # get all "amenities" and save as a geopackage via geopandas
    #     # gdf = ox.geometries_from_place(place, tags=tags)
    #     # gdf = G.apply(lambda c: c.astype(str) if c.name != "geometry" else c, axis=0)
    #     # gdf.to_file(f"{config.input_gpkg_file_path}/example.gpkg", driver="GPKG")
    #     # G.to_file(config.input_gpkg_file_path+"example.gpkg", driver="GPKG")
    #
    #     # ox.save_graph_geopackage(G, filepath=config.input_gpkg_file_path+"example.gpkg")
    #     # # filepath = "./data/piedmont.graphml"
    #     # ox.save_graphml(G, edinburgh_graph_filepath)

    # place = "Edinburgh, UK"
    # G = ox.graph_from_place(place, network_type='drive')
    # use networkx to calculate the shortest path between two nodes
    #
    #
    # # origin_node = list(G.nodes())[0]
    # # destination_node = list(G.nodes())[-12]
    # # print(origin_node,destination_node)
    # route = nx.shortest_path(G, origin_node, destination_node)
    # print(route)
    # m1 = ox.plot_graph_folium(G, popup_attribute="name", weight=2, color="#8b0000")
    # m2 = ox.plot_route_folium(G, route, weight=10)
    # m3 = ox.plot_route_folium(G, route, route_map=m1, popup_attribute="length", weight=7)

    # find the shortest path (by distance) between these nodes then plot it
    # orig = list(G.nodes)[0]
    # dest = list(G)[120]
    # route = ox.shortest_path(G, orig, dest, weight="length")
    # fig, ax = ox.plot_graph_route(G, route, route_color="y", route_linewidth=6, node_size=0)
    # cx.add_basemap(ax, crs=G.crs.to_string())
    # filepath = "data/graph.html"
    # m1.save(filepath)
    # fig, ax = ox.plot_graph(G)

# route = ox.shortest_path(G, orig, dest, weight="length")
#
# N=G.nodes[dest]['y']
# S=G.nodes[orig]['y']
# E=G.nodes[dest]['x']
# W=G.nodes[orig]['x']

# back_img, extents = cx.bounds2img(W, S, E, N, ll=True, source=cx.providers.CartoDB.Voyager)
