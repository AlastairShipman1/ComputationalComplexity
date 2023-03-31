import math
from pyproj import Transformer
from PIL import Image
import requests
import time
import config
import numpy as np


# Built-in gps to mercator (and vice-versa) converter functions from pyproj, documentation available.
def gps_2_merc(lat, long):
    transformed = Transformer.from_proj("EPSG:4326", "EPSG:3857").transform(lat, long)
    return transformed


def merc_2_gps(merc_x, merc_y):
    transformed = Transformer.from_proj("EPSG:3857", "EPSG:4326").transform(merc_x, merc_y)
    return transformed


def generate_background(limits, origin):
    """Generates a background based on data limits.
    The maps are taken from OSM tiles, and stitched together appropriately. """

    # currently the padding is in metres.
    padding_factor = 0#config.visualisation_metrics["figure_padding"]

    min_x = limits['min_x'] - padding_factor
    min_y = limits['min_y'] - padding_factor
    max_x = limits['max_x'] + padding_factor
    max_y = limits['max_y'] + padding_factor

    bound_mins = (min_x, min_y)
    bound_maxes = (max_x, max_y)

    # bound_ll -> lower left boundary (bottom left corner of limits)
    # bound_ur -> upper right boundary (top right corner of limits)

    origin_merc = gps_2_merc(*origin)  # the listed origin point of the world, unpack as a tuple

    # Get the usable boundaries of the data,
    bound_ll_merc = np.add(origin_merc, bound_mins)
    bound_ur_merc = np.add(origin_merc, bound_maxes)

    # Convert the values to gps for use with OSM
    bound_ll_gps = merc_2_gps(*bound_ll_merc)
    bound_ur_gps = merc_2_gps(*bound_ur_merc)

    bound_limits = (*bound_ll_gps, *bound_ur_gps)

    # Save the image
    filepath = config.input_image_file_path
    background_from_limits(bound_limits, filepath + 'bg_low_qual.png')


def background_from_limits(data_extents, path):
    """Get a tiled image from OSM based on calculated extents, save it to path."""

    minlat, minlon, maxlat, maxlon = data_extents

    ll = (minlat, minlon)
    ul = (maxlat, minlon)
    lr = (minlat, maxlon)
    ur = (maxlat, maxlon)

    corners = (ll, ul, lr, ur)

    # Some of this stuff should be changed depending on the tile provider.
    # Minimum adjacent tiles can force a higher resolution if available.
    # min_adjacent_tiles = 1
    # zoom = get_zoom(min_adjacent_tiles, corners)  # Legacy calculator from UST0.

    zoom = 16  # Force highest resolution here... 17 is fine, 19 is highest?(or28?)
    x_range, y_range = get_tile_ranges(zoom, corners)
    im, im_extents = get_base_image(x_range, y_range, zoom)
    im.save(path)


# Converter functions for OSM tiles, documentation available from OSM
# This is a strippd-down version of the UST0 code. Largely as the GPS-Mercator code is handled by pyproj

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)


def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)


def bounds(xtile, ytile, zoom):
    up, left = num2deg(xtile, ytile, zoom)
    down, right = num2deg(xtile + 1, ytile + 1, zoom)
    return up, down, left, right


# Tile functionality
def get_zoom(num_tiles, corners):
    ll, ul, lr, ur = corners

    zoom = 0
    while zoom <= 18:  # Max zoom is 19
        ll_x, ll_y = deg2num(*ll, zoom)
        ul_x, ul_y = deg2num(*ul, zoom)
        lr_x, lr_y = deg2num(*lr, zoom)

        x_diff = abs(ll_x - lr_x)
        y_diff = abs(ll_y - ul_y)

        diffs = [x_diff, y_diff]

        if all([diff >= num_tiles for diff in diffs]):
            break
        else:
            zoom += 1

    return zoom


def get_tile_ranges(zoom, corners):
    ll, ul, lr, ur = corners
    ll_x, ll_y = deg2num(*ll, zoom)
    ur_x, ur_y = deg2num(*ur, zoom)

    if ll_x <= ur_x:
        x_range = range(ll_x, ur_x + 1)
    else:
        x_range = range(ur_x, ll_x)

    if ll_y <= ur_y:
        y_range = range(ll_y, ur_y + 1)
    else:
        y_range = range(ur_y, ll_y)

    return x_range, y_range


# Image processing
def concat_img_y(base, new):
    strip = Image.new('RGB', (base.width, base.height + new.height))
    strip.paste(base, (0, 0))
    strip.paste(new, (0, base.height))
    return strip


def concat_img_x(base, new):
    strip = Image.new('RGB', (base.width + new.width, base.height))
    strip.paste(base, (0, 0))
    strip.paste(new, (base.width, 0))
    return strip


def get_base_image(x_range, y_range, zoom):
    """Gets the base OSM tiles, and stitches them together using PIL"""
    lats = []
    longs = []

    print("Getting background tiles...")
    # Get tile for each pair of xy coordinates in OSM at a specific zoom level
    for i, x in enumerate(list(x_range)):
        for j, y in enumerate(list(y_range)):
            url = f'http://tile.openstreetmap.org/{zoom}/{x}/{y}.png'  # Default
            # url = f'https://a.tiles.wmflabs.org/osm-no-labels/{zoom}/{x}/{y}.png' # No labels
            url = f'http://a.tile.openstreetmap.fr/hot//{zoom}/{x}/{y}.png' # Alternative visual style

            while True:
                try:
                    url_image_raw=requests.get(url, stream=True)
                    print(i, j, url_image_raw)
                    tile = Image.open(url_image_raw.raw)
                    break
                except:
                    print("Error, retrying in 5 seconds:")
                    time.sleep(5)
            # Stitch together columns first, then rows as appropriate. Update data at the same time.

            if j == 0:
                strip = tile
            else:
                strip = concat_img_y(strip, tile)

            up, down, left, right = bounds(x, y, zoom)

            lats.extend([up, down])
            longs.extend([left, right])

        if i == 0:
            im = strip
        else:
            im = concat_img_x(im, strip)

    # Calculate new extents
    if len(lats) >= 1:
        min_lat = min(lats)
        max_lat = max(lats)
    if len(longs) >= 1:
        max_long = max(longs)
        min_long = min(longs)

    extents = min_lat, min_long, max_lat, max_long

    return im, extents

    # def add_tuples(*list_of_tuples):
    #     """Helper function to add tuples together, I know this is stupid but everything was formatted as a tuple here
    #     and it was this or do shit tons of converting between types. """
    #     if len(list_of_tuples) == 2:
    #         added = np.add(*list_of_tuples)
    #     elif len(list_of_tuples) > 2:
    #         added = add_tuples(add_tuples(list_of_tuples[0], list_of_tuples[1]), *list_of_tuples[2:])
    #     return tuple(added)
    #
    # def scale_tuples(scale_val, *tuples):
    #     """Helper function to scale values in a tuple, I know this is also stupid and non-Pythonic but it's done now.
    #     """
    #     if len(tuples) == 1:
    #         values = tuples[0]
    #         return tuple([val * scale_val for val in values])
    #     else:
    #         return tuple(tuple([val * scale_val for val in values]) for values in tuples)
