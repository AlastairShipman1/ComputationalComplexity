

# need to do bubble generation
# from a list of waypoints, and a list of obstacles
# adapted for time-specific locations

# then do a energy optimisation
# combine both of these with biomechanical aspects of movement (energy expenditure minimisation)
#

'''

1) CREATE BUBBLE PATH
reference trajectory P, made of n waypoints
each step has a bubble, B[i], with an associated centre, A[i], and radius, r[i]
obstacle set O
bubble bounds r_u and r_l

for i in range(2, n-1):
    if abs(P[i] -A[i-1])< 0.5*r[i-1]:
        B[i]=B[i-1]
        A[i]=A[i-1]
        r[i]=r[i-1]
        continue
    B[i]=GenerateBubble(P[i].centre)
    r[i]=B[i].radius
    A[i]=P[i]
    if r[i]<r_l:
        B[i]=TranslateBubble(A[i], r[i])
        r[i]=B[i].radius
        A[i]=B[i].centre



def GenerateBubble(point):
    return largest circle that is collision free, based on the local obstacles
    that has a lower radius than r_u

    For rectangular-shaped obstacles, this is
    a straightforward geometrical procedure. If the obstacles
    have more general polygonal shapes, a bisection search is
    performed with respect to the bubble radius.

def TranslateBubble(point, radius):

    first: id the edge of the obstacle closest to Ai
    Then:, the outward normal direction to the edge is computed and the center of the
    bubble is moved along such direction until a ball of radius
    rl can be placed.

    Should this not be possible, then
    TranslateBubble(Ai; ri) returns the ball of largest radius
    among the balls whose centers lie on the aforementioned
    normal direction.


2) OPTIMISE ACTUAL TRAJECTORY WITHIN BUBBLE PATH, USING STRETCHING AND SPEED OPTIMISATION


3) REPEAT 2 UNTIL ONE OF THE FOLLOWING CONDITIONS IS MET:
    - IT DOESN'T GET BETTER (TIME, OR DISTANCE REDUCTIONS)
    - TOO MANY RECURSIVE CALLS HAVE BEEN MADE

4) should include total energy reductions as well in 2).







STRETCHING:
we have waypoints, and bubble region
The parameters for the model in equations are m = 833 kg, mu=0:8, and u_long = 0.5 mu*m*g.
ru = 10 m and rl = 1 m


if you imagine the connection between waypoints as a force balance, then the stretching
requires us to add and remove waypoints, all within the bubble region,
to minimise the sum of the forces required (sum(N[k] for all k))

note: theta[k] is the angle subtended by the vectors Q[k-1:k] and Q[k:k+1]

The constraints for the placement of the new waypoints rely on a number of approximations to ensure convexity of
the optimization problem. Specifically, assume the longitudinal force
    u_long[k]
and velocity
    v[k]
are given at each waypoint in P (their optimization will be discussed in the next section).
We define R[k] as the instantaneous turning radius for the agent at the kth waypoint.
The lateral acceleration a_lat for waypoints Q[k], k = 2...n - 1 can be upper bounded as
    a[k]= |v[k]|**2/R[k] <=np.sqrt( (mu*g)**2 - (u_long[k]/m)**2 ) = alpha[k]

hence
    1/R[k]<= alpha[k]/|v[k]|**2

using the following approximations (which hold for dense, smooth waypoints)
    |F[k]|~=|F[k-1]|
    theta[k] is small
    theta[k]~= |Q[k+1]-Q[k]|/R[k]

maths
    N[k]<=|Q[k+1]-Q[k]|**2 *alpha[k]/|v[k]|**2

finally, approximate the length of each band/edge:
    d=sum(distance(p[k+1],p[k]) for all k)/n-1

    |N[k]|=alpha[k]*((d/|v[k]|)**2)

    N[k]<=d**2/R_min

then, constrain the beginning and end of the trajectory: Q[0]=P[0], Q[n-1]=P[n-1]
In turn, to constrain the initial and final heading angles, one can use the
constraints Q[2] = P[1]+d(v1/|v1|)
ond therefore:
    Q[n-1] = P[n]-d*v[n-1]/|v[n-1]|

Noting that N[k] = 2*Q[k]-Q[k-1]-Q[k+1], the elastic stretching
optimization problem is:
min over Q[3...n-2]
    sum of N[k]**2

such that   Q[0]=P[0], Q[1]=P[0]+d*v[1]/|v[1]|
and         Q[n]=P[n], Q[n-1]=P[n]-d*v[n-1]/|v[n-1]|

and Q[k] lies within B[k]

and |N[k]| <= min(d^2/R_min, alpha[k]*(d/v[k])**2)




SPEED OPTIMISATION: unnecessary if speed is fixed.


The speed optimization over a fixed trajectory relies on
the convex optimization algorithm presented [speed_optimisation_paper].
The inputs to this algorithm are
    (1) a sequence of waypoints
    (2) the friction coefficient for the friction circle constraint (change this for lat vs long?)
    (3) the maximum traction force u_long

The outputs are (1) the sequence of velocity vectors (2) the sequence of
longitudinal control forces ulong[k], one for each waypoint Q[k]

Algorithm 6.1 Speed optimisation algorithm
Given the d(external forces), vehicle path s, constraint set C and initial velocity v[0]

Discretise path s at n + 1 points get the time vector of those discretised waypoints, T

theta(t) takes the trajectory waypoint times (0 to T), and maps them (0 to 1)
s(theta) maps out the trajectory (i.e. q(t)), using this standardised time
b[theta]=theta_dot **2
d_theta= theta[k+1]-theta[k] # constant across 0 to 1?

        minimise: 2*sum((theta[k+1]-theta[k])/(np.sqrt(b[k])+np.sqrt(b[k+1]))
                k=0..n-1


subject to:
        theta[ave]=0.5*(theta[k]+theta[k-1])
        b[ave]=-.5*(b[k]+b[k-1])

        force[theta[k]]=m[theta[k]]*a[k]
                            +c[theta[k]]b[ave]
                            +d[theta[ave]]
                k=1..n

        force[theta[k]] ≤ u_long_max (=WfμsFN)

        a[theta]=theta_dotdot
        b'[theta]=2a[theta]

        a[k],b[k] are elements of constraints[theta[k]]
            k=1...n

        b[0]=(|v[0]| *d_theta / |s[theta[1]] -s[theta[0]]|)**2 = alpha**2

        b[i]-b[i-1]=2*alpha[i]*d_theta




Create barrier functions for the constraint set C.
Choose a starting point for ai , bi , ui , that satisfies the constraint set C, bi > 0.
In most cases, bi small and positive, ui and ai zero will satisfy these constraints.
(5) Solve the infeasible start interior point problem.
Create KKT systems that interweave the blocks
of (43) and (44).
Solve the KKT systems using anLU factorisation
for solve times linear in n.

'''


#2) OPTIMISE ACTUAL TRAJECTORY WITHIN BUBBLE PATH, USING STRETCHING AND SPEED OPTIMISATION


#3) REPEAT 2 UNTIL ONE OF THE FOLLOWING CONDITIONS IS MET:
#    - IT DOESN'T GET BETTER (TIME, OR DISTANCE REDUCTIONS)
#    - TOO MANY RECURSIVE CALLS HAVE BEEN MADE

#4) should include total energy reductions as well in 2).

from dataclasses import dataclass, field
import copy
import shapely.geometry.polygon
from shapely.geometry import LineString, Polygon, Point, LinearRing, MultiPolygon
from shapely.ops import nearest_points

from matplotlib.patches import Polygon as mPolygon
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from matplotlib.patches import Wedge
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib as mpl
import cvxpy as cp
import numpy as np


from typing import List
import numpy as np
import config

@dataclass
class Bubble:
    radius: float
    timestamp: float
    direction: float
    centre: List[float] = field(default_factory=List)
    acceleration=0
    speed=0

@dataclass
class Waypoint:
    timestamp: float
    speed: float
    direction: float
    centre: List[float]= field(default_factory=List)

r_u, r_l = 1.5, .5


def FindLeastRadiusFromObstacles(point, obstacles):
    radius = 999
    nearest_obstacle = obstacles[0]
    for i in obstacles:
        tp=Point(point)
        if i.contains(tp):
            # move the waypoint away from the centre of the obstacle?
            # the main route should never collide with an obstacle, but the dynamic route might (esp with predictions).
            #
            return 0, i
            #raise Exception("Point lies within an obstacle")
        dist_from_edge = i.distance(tp)
        if dist_from_edge < radius:
            radius = dist_from_edge
            nearest_obstacle = i
    return radius, nearest_obstacle

def GenerateBubble(waypoint, obstacles):
    #If the obstacles have more general polygonal shapes, a bisection search is
    #performed with respect to the bubble radius.
    point=waypoint.centre
    radius, nearest_obstacle=FindLeastRadiusFromObstacles(point, obstacles)
    timestamp=waypoint.timestamp
    if radius< r_l:
        radius, point=TranslateBubble(radius, waypoint, nearest_obstacle, obstacles)
    radius = min(radius, r_u)
    direction=waypoint.direction
    return Bubble(radius, timestamp, direction, point)

def GetUnitNormalFromObstacle(point, obstacle):
    """
    get the direction away from the centroid of the polygon
    """
    centre = obstacle.centroid.coords[0]
    grad = [point[0] - centre[0], point[1] - centre[1]]
    return grad / np.linalg.norm(grad)

def TranslateBubble(radius, waypoint, nearest_obstacle, obstacles):
    initial_radius=radius
    point=waypoint.centre
    direction= GetUnitNormalFromObstacle(point, nearest_obstacle)

    while radius<r_l:
        point= [point[0]+direction[0]*.1, point[1]+direction[1]*.1]
        radius, nearest_obstacle = FindLeastRadiusFromObstacles(point, obstacles)

        if radius< initial_radius:
            # can occur with weird, non-convex polygons- reverse the direction, add some randomness, and hope that helps.
            direction=[-direction[1]+0.1*np.random.rand()[0], direction[0]+0.1*np.random.rand()]
            initial_radius=radius
    return radius, point

def RingCoding(ob):
    # The codes will be all "LINETO" commands, except for "MOVETO"s at the
    # beginning of each subpath
    n = len(ob.coords)
    codes = np.ones(n, dtype=Path.code_type) * Path.LINETO
    codes[0] = Path.MOVETO
    return codes

def Pathify(polygon):
    # Convert coordinates to path vertices. Objects produced by Shapely's
    # analytic methods have the proper coordinate order, no need to sort.
    vertices = np.concatenate(
                    [np.asarray(polygon.exterior)]
                    + [np.asarray(r) for r in polygon.interiors])
    codes = np.concatenate(
                [RingCoding(polygon.exterior)]
                + [RingCoding(r) for r in polygon.interiors])
    return Path(vertices, codes)

def CreatePatch(poly,area_override=None):
    MAX_DENSITY = 0.75
    area = poly.area
    if area_override is not None:
        area=area_override
    density = 1 / area
    color = (min(1, density / MAX_DENSITY), max(0, (MAX_DENSITY - density) / MAX_DENSITY), 0, 0.5)
    region_external_coords = list(poly.exterior.coords)

    if len(poly.interiors) > 0:
        path = Pathify(poly)
        patch = PathPatch(path)
    else:
        patch = mPolygon(region_external_coords)
    patch.set_color(color)
    return patch

def PointPointDistance(p1, p2):
    dx=p1[0]-p2[0]
    dy=p1[1]-p2[1]
    return np.sqrt(dx**2+dy**2)

def _convert_continuous_to_waypoints(continuous_route):
    speed = 1
    lines = []
    waypoints=[]

    for i in range(len(continuous_route.xy[0]) - 1):
        line = LineString([(continuous_route.xy[0][i], continuous_route.xy[1][i]),
                           (continuous_route.xy[0][i + 1], continuous_route.xy[1][i + 1])])
        lines.append(line)

    line_start_time=0
    for i in lines:
        dist = i.length
        duration=dist/speed
        # should make this linked to ticksteps, and then the leftover time should also accommodate
        tick_steps = int(duration*10)
        leftover_time=duration%.1
        dx=i.coords[1][0]-i.coords[0][0]
        dy=i.coords[1][1]-i.coords[0][1]

        direction = np.arctan2(dy, dx)

        for j in range(tick_steps):
            interpolation_value = j / tick_steps
            interp_point = i.interpolate(interpolation_value*dist)
            timestamp = line_start_time + interpolation_value * duration
            temp_waypoint = Waypoint(timestamp, speed,direction, [interp_point.xy[0][0], interp_point.xy[1][0]])
            waypoints.append(temp_waypoint)
        line_start_time=timestamp+leftover_time
    return waypoints


def PlotEverything(bubbles_list, outline_poly, points_list, optimised_positions_list):
    #[bubbles, bubbles_1], outline_poly, [points, route_2], [x, y]
    fig, ax= plt.subplots()
    patch=CreatePatch(outline_poly)
    ax.add_patch(patch)

    rings=[]
    for i in bubbles_list:
        for bubble in i:
            ring= Wedge(bubble.centre,bubble.radius, 0, 360, width=0.05)  # Full ring
            rings.append(ring)
            plt.scatter(bubble.centre[0],bubble.centre[1], c='blue', alpha=0.4)

    for optimised_positions in optimised_positions_list:
        for i in range(len(optimised_positions[0])):
            plt.scatter(optimised_positions[0][i], optimised_positions[1][i], c='red', alpha=0.4)

    p = PatchCollection(rings, alpha=0.4)
    ax.add_collection(p)
    bounds = outline_poly.bounds

    for points in points_list:
        planned_coords = list(zip(*points))
        plt.plot(planned_coords[0], planned_coords[1])

    ax.set_xlim(bounds[0]- 1,bounds[2] + 1)
    ax.set_ylim(bounds[1] - 1, bounds[3] + 1)
    plt.show()

def FindAverageLength(bubbles):
    d=0
    for i in range(1,len(bubbles)-1):
       d+= PointPointDistance(bubbles[i].centre, bubbles[i-1].centre)
    d/=(len(bubbles)-1)
    return d

def OptimiseForce(bubbles):
    n=len(bubbles)
    x = cp.Variable((2, n))

    objective = 0
    d=FindAverageLength(bubbles)

    constr = [x[:, 0] == (bubbles[0].centre[0],bubbles[0].centre[1]),
              x[:, 1] == (bubbles[1].centre[0],bubbles[1].centre[1]),
              x[:, n - 2] == (bubbles[n-2].centre[0],bubbles[n-2].centre[1]),
              x[:, n-1] == (bubbles[n-1].centre[0],bubbles[n-1].centre[1])]
    for k in range(2, n - 2):
        objective += cp.sum_squares(2 * x[:, k] - x[:, k - 1] - x[:, k + 1])
        # need to add one final constraint- alpha_k d**2/v_k. need to add a speed and acceleration for each bubble.
        constr.append(cp.norm(2 * x[:, k] - x[:, k - 1] - x[:, k + 1])
                      <= min(d**2/1,bubbles[k].acceleration*(d/bubbles[k].speed)**2))

        constr.append(cp.norm(x[:, k] - bubbles[k].centre) <= bubbles[k].radius)
    problem = cp.Problem(cp.Minimize(objective), constr)# The optimal objective value is returned by `prob.solve()`.
    result = problem.solve()
    return x.value

def FindClosestWaypointTime(timestamp, waypoints):
    closest_waypoint=waypoints[0]
    d_t = abs(closest_waypoint.timestamp-timestamp)

    for wp in waypoints:
        temp_d_t = abs(wp.timestamp - timestamp)
        if temp_d_t < d_t:
            closest_waypoint = wp
            d_t = temp_d_t
    return closest_waypoint

def CreateBubbles(waypoints, obstacles, waypoints_other):
    bubbles=[]

    bubbles.append(GenerateBubble(waypoints[0], obstacles))
    for i in range(1, len(waypoints)):
        closest_wp_other = FindClosestWaypointTime(waypoints[i].timestamp, waypoints_other)
        d_wp = PointPointDistance(closest_wp_other.centre, waypoints[i].centre)

        d = PointPointDistance(waypoints[i].centre, bubbles[-1].centre)
        cutoff = .5*bubbles[-1].radius
        if d < cutoff and d_wp > 2*r_u:
            continue
        else:
            dynamic_obstacle = Point(closest_wp_other.centre).buffer(1)
            temp_obstacles=copy.copy(obstacles)
            temp_obstacles.append(dynamic_obstacle)
            b=GenerateBubble(waypoints[i], temp_obstacles) # generate bubble automatically calls translate bubble
            d = PointPointDistance(b.centre, bubbles[- 1].centre)
            if d < cutoff:
                continue
            bubbles.append(b)

    for i in range(1,len(bubbles)-1):
        ds_pre = PointPointDistance(bubbles[i].centre, bubbles[i-1].centre)
        dt_pre = bubbles[i].timestamp-bubbles[i-1].timestamp
        ds_post = PointPointDistance(bubbles[i].centre, bubbles[i + 1].centre)
        dt_post = bubbles[i].timestamp - bubbles[i + 1].timestamp

        bubbles[i].speed = 0.5 * (ds_pre/dt_pre + ds_post/dt_post)
        dtheta = bubbles[i].direction - bubbles[i-1].direction

        dv = bubbles[i].speed-bubbles[i-1].speed
        dv_lat = dv*np.sin(dtheta)
        dv_long = dv * np.cos(dtheta)
        bubbles[i].acceleration = (np.sqrt(dv_long**2+dv_lat**2))/dt_pre

    return bubbles


def main():
    # create environment
    a = [(-40, -4), (-40, 6), (2, 6), (2, -4), (-40, -4)]
    hole_a_1 = LinearRing([(-20, -1.5), (-25, -2.5), (-25, -2), (-20, 2), (-20, -1.5)])
    hole_a_2 = LinearRing([(-30, -2), (-35, -2), (-35, 2), (-30, 2), (-30, -2)])
    hole_a_3 = LinearRing([(-20, 4.9), (-10, 4.9), (-10, 2), (-20, 4.9)])
    outline_poly = Polygon(a, [hole_a_1, hole_a_2, hole_a_3])
    outline_poly = shapely.geometry.polygon.orient(outline_poly)
    obstacle_1 = Polygon(hole_a_1)
    obstacle_2 = Polygon(hole_a_2)
    obstacle_3 = Polygon(hole_a_3)

    # a list of the obstacles in the environment
    obstacles = [obstacle_1, obstacle_2, obstacle_3]

    route_1 = [[-10, -3], [-15, 1.5], [-20, 4], [-37, 3.5], [-35, -2.5]]
    route_1 = [[0, 5.5], [0, -3.5]]
    continuous_route_1 = LineString(route_1)
    route_2 = [[-15, 2.5], [-10, -1], [-30, -2.5], [-39, -3], [-37, 2.3], [-17.5, 2.5]]
    route_2 = [[-0.5, -3.5], [-0.5, 5.5]]
    continuous_route_2 = LineString(route_2)

    waypoints_1 = _convert_continuous_to_waypoints(continuous_route_1)
    waypoints_2 = _convert_continuous_to_waypoints(continuous_route_2)

    # take in a list of all other waypoints, find minimum timestamp difference, and treat it as an obstacle
    bubbles_1 = CreateBubbles(waypoints_1, obstacles, waypoints_2)
    bubbles_2 = CreateBubbles(waypoints_2, obstacles, waypoints_1)

    optimised_1 = OptimiseForce(bubbles_1)
    optimised_2 = OptimiseForce(bubbles_2)
    PlotEverything([bubbles_2, bubbles_1], outline_poly, [route_2, route_1], [optimised_2, optimised_1])
    #PlotEverything([bubbles_2], outline_poly, [route_2], [optimised_2])


    #PlotEverything([], outline_poly, [route_2, route_1], [optimised_2, optimised_1])

if __name__ == '__main__':
    main()
