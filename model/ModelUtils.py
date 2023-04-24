import numpy as np


class Waypoint:
    '''
        basic waypoint type
        this class defines the point that an agent should get to,
        at a time they should be there, and the velocity that they should have when they are
    '''
    def __init__(self,guid, x, y, v_x=0, v_y=0, theta=0):
        self.position = [x, y]
        self.guid = guid
        self.velocity = [v_x, v_y]
        self.direction = theta


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

