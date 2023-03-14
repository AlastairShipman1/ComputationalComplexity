"""
it is clearly stated that osmnx should only really be used with conda.
currently you have a python virtual environment
not a conda venv.
this will cause issues.

import osmnx as ox
import pandas
import geopandas
import momepy
import networkx as nx
import contextily
import matplotlib.pyplot as plt
from pysal.viz import mapclassify as mc



hyde_park= ox.geometries_from_place("Hyde Park, London", tags={"leisure":"park"})
hyde_park.plot()

canals=ox.geometries_from_place("London", tags={"waterway":"canal"})
canals.plot()
canal_buffer=canals.to_crs(epsg=27700).buffer(50)
#buffer, latlong_crs = ox.project_geometry(projected_buffer, crs=projection_crs, to_latlong=True)
#G = ox.graph_from_polygon(buffer)
canal_buffer.plot()
cafes=ox.geometries_from_polygon(canal_buffer.to_crs(epsg=4326).unary_union, tags={"amenity":"cafe"})
ax=canal_buffer.to_crs(epsg=4326).plot()
cafes.plot(ax=ax,
    facecolor="none", edgecolor="blue", linewidth=2
)

contextily.add_basemap(
    ax,
    crs=canal_buffer.crs,
    source=contextily.providers.OpenStreetMap.Mapnik
);



db = geopandas.read_file("./Data/arturo_streets.gpkg")
db_tab = db.explode().reset_index()
db_tab.head()
ax=db_tab.plot(color='black', linewidth=0.1,figsize=(12, 12))
length = db_tab.to_crs(
    epsg=32630 # Expressed in metres
).geometry.length
length.head()

ax = db_tab.assign(
    length=length
).plot(
    "length",
    scheme="fisherjenkssampled",
    k=9,
    legend=True,
    linewidth=0.5,
    figsize=(12, 12),
    cmap="magma"
)
contextily.add_basemap(
    ax,
    crs=db_tab.crs,
    source=contextily.providers.CartoDB.PositronNoLabels,
    alpha=0.95
)
ax.set_title("Street segment length");

f, ax= plt.subplots()

col="length"
classi1 = mc.FisherJenks(db_tab[col], k=8)

sns.kdeplot(db_tab[col], fill=True, ax=ax,)

for i in range(0, len(classi1.bins)-1):
    ax.axvline(classi1.bins[i], color="red")


db_tab["length"]=length
linearity = momepy.Linearity(db_tab).series

ax = db_tab.assign(
    linearity=linearity
).plot(
    "linearity",
    scheme="fisherjenkssampled",
    k=9,
    legend=True,
    linewidth=0.5,
    figsize=(12, 12),
    cmap="magma"
)
contextily.add_basemap(
    ax,
    crs=db_tab.crs,
    source=contextily.providers.CartoDB.PositronNoLabels,
    alpha=0.5
)
ax.set_title("Street segment linearity");
db_graph = momepy.gdf_to_nx(db_tab)
db_graph.is_directed()
db_graph.is_multigraph()
l = db_tab.loc[0, "geometry"]
l.coords

node0a, node0b = edge0 = list(
    db_tab.loc[0, "geometry"].coords
)
edge0
db_graph[node0a]
db_graph[node0a][node0b][0]
list(
    db_graph.nodes
)[:5]

list(
    db_graph.edges
)[:5]


%time meshd = momepy.meshedness(db_graph, distance=500)

meshd.nodes[node0a]
pandas.Series(
    {i: meshd.nodes[i]["meshedness"] for i in meshd.nodes}
).plot.hist(bins=100, figsize=(9, 4));



ax = nodes.plot(
    "meshedness",
    scheme="fisherjenkssampled",
    markersize=0.1,
    legend=True,
    figsize=(12, 12)
)
contextily.add_basemap(
    ax,
    crs=nodes.crs,
    source=contextily.providers.CartoDB.DarkMatterNoLabels
)
ax.set_title("Meshedness");

degree_tab = pandas.DataFrame(
    degree, columns=["id", "degree"]
)
degree_tab.index = pandas.MultiIndex.from_tuples(
    degree_tab["id"]
)
degree_tab = degree_tab["degree"]
degree_tab.head()


net_stats = pandas.DataFrame(
    {"degree": degree_tab, "centrality": nc},
)
net_stats.index.names = ["x", "y"]
net_stats.head()

nc = pandas.Series(
    nx.degree_centrality(db_graph)
)
nc.head()

net_stats_geo = nodes.assign(
    x=nodes.geometry.x
).assign(
    y=nodes.geometry.y
).set_index(
    ["x", "y"]
).join(net_stats)

net_stats_geo.head()

f, axs = plt.subplots(1, 2, figsize=(18, 9))
vars_to_plot = ["degree", "centrality"]
for i in range(2):
    net_stats_geo.plot(
        vars_to_plot[i],
        scheme="fisherjenkssampled",
        markersize=0.2,
        legend=True,
        ax=axs[i]
    )
    contextily.add_basemap(
        axs[i],
        crs=nodes.crs,
        source=contextily.providers.CartoDB.DarkMatterNoLabels
    )
    axs[i].set_title(f"Node {vars_to_plot[i]}")


roads=ox.geometries_from_address("Skempton Building, South Kensington, London", tags={"highway":"road"}, dist=200)
centro_gr = ox.graph_from_polygon(roads.squeeze().geometry)
ox.plot_graph_folium(roads)


from shapely.geometry import LineString

def route_nodes_to_line(nodes, network):
    pts = network.nodes_df.loc[nodes, :]
    s = geopandas.GeoDataFrame(
        {"src_node": [nodes[0]], "tgt_node": [nodes[1]]},
        geometry=[LineString(pts.values)],
        crs=streets.crs
    )
    return s

import pandana



streets = geopandas.read_file("./Data/arturo_streets.gpkg")
abbs = geopandas.read_file("./Data/madrid_abb.gpkg")
neis = geopandas.read_file("./Data/neighbourhoods.geojson")
%%time
nodes, edges = momepy.nx_to_gdf(
    momepy.gdf_to_nx(
        streets.explode() # We "explode" to avoid multi-part rows
    )
)
nodes = nodes.set_index("nodeID") # Reindex nodes on ID

streets_pdn = pandana.Network(
    nodes.geometry.x,
    nodes.geometry.y,
    edges["node_start"],
    edges["node_end"],
    edges[["mm_len"]]
)
## mm_len is the friction measurement.

streets_pdn




import geopy
geopy.geocoders.options.default_user_agent = "gds4ae"
sol = geopandas.tools.geocode(
    "Puerta del Sol, Madrid", geopy.Nominatim
).to_crs(streets.crs)
sol

pt_nodes = streets_pdn.get_node_ids(
    [first.geometry.x, sol.geometry.x],
    [first.geometry.y, sol.geometry.y]
)
pt_nodes

route_nodes = streets_pdn.shortest_path(
    pt_nodes[0], pt_nodes[1]
)
route_nodes


route = route_nodes_to_line(route_nodes, streets_pdn)

route_len = streets_pdn.shortest_path_length(
    pt_nodes[0], pt_nodes[1]
)
round(route_len / 1000, 3) # Dist in Km


# network distance from each airbnb to an electric vehicle charging point

electric_vehicles_cps = ox.geometries_from_place("Madrid, Spain", tags={"amenity": "charging_station"}).to_crs(streets.crs)


electric_vehicles_cps.plot()

streets_pdn.set_pois(
    category="Charging Points",
    maxitems=1,
    maxdist=100000, # 100km so everything is included
    x_col=electric_vehicles_cps.geometry.x,
    y_col=electric_vehicles_cps.geometry.y,
)


charging_station_2nnode = streets_pdn.nearest_pois(
    100000,              # Max distance to look for
    "Charging Points",    # POIs to look for
    num_pois=1,          # No. of POIs to include
    include_poi_ids=True # Store POI ID
# Then add the charging point IDs and name
).join(
    electric_vehicles_cps[["unique_id", "osmid", "name"]],
    on="poi1"
# Rename the distance from node to charging point
).rename(
    columns={1: "dist2charging_point"}
)
charging_station_2nnode.info()

abbs_nnode = streets_pdn.get_node_ids(
    abbs.to_crs(streets.crs).geometry.x, abbs.to_crs(streets.crs).geometry.y
)
abbs_nnode.describe()

electric_vehicles_cps.describe()

abb_charging_point = abbs.to_crs(streets.crs)[["geometry"]].assign(
    nnode=abbs_nnode).join(
    charging_station_2nnode, on="nnode"
)
abb_charging_point.head()


london_roads=ox.geometries_from_address("London, UK", tags={"highway":True}, dist=1000)
madrid_parks=ox.geometries_from_place("Madrid, Spain", tags={"leisure":"park"})
"""



