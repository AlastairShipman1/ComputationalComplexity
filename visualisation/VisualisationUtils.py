import numpy as np
from pygame import Color

class Colours:
    BLACK = 0, 0, 0
    GREY = 192, 192, 192
    GREEN = 0,255,0
    BLUE = 0, 0, 255
    WHITE = 255, 255, 255
    RED = 255, 0, 0
    ORANGE = 255, 140, 0
    PURPLE = 128, 0, 128
    YELLOW = 255, 255, 0
    CLEAR = Color(0,0,0,0)



def is_left(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 0


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    mag = np.linalg.norm(vector)
    if mag == 0:
        return [0, 0]
    return vector / mag


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

