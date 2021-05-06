import csv
from math import pi
import numpy as np
from math import cos, asin, sqrt, pi,atan2, sin
import math

K_THRESH = .1

def norm(x):
    sqnorm = x[0] * x[0] + x[1] * x[1]
    return math.sqrt(sqnorm)


# simple dot product
def dot_len2(x, y):
    return x[0] * y[0] + x[1] * y[1]


def dot_Matrix_Vector(M2, V2):
    return [M2[0][0] * V2[0] + M2[0][1] * V2[1], M2[1][0] * V2[0] + M2[1][1] * V2[1]]


# Stackoverflow: dot product of two vectors in python 3 with out using the sum or zip function
def dot_any_len(vector01, vector02):
    if len(vector01) != len(vector02):
        raise ValueError
    total = 0
    for i in range(len(vector01)):
        total += vector01[i] * vector02[i]
    return total


def get_heading(a, b):
    y_diff = b[1] - a[1]
    x_diff = b[0] - a[0]

    return atan2(y_diff, x_diff)


def get_angle(p1, p2, p3):
    a1 = atan2(p3[1] - p2[1], p3[0] - p2[0])
    a0 = atan2(p2[1] - p1[1], p2[0] - p1[0])
    d_ang = a1 - a0
    d_ang = atan2(sin(d_ang), cos(d_ang))
    return d_ang

def euc(x,y):
    distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))
    return distance


def compute_cte(current_pos, waypoints, current_heading):

    prev_pos_ind = closest_node_behind(current_pos, waypoints, current_heading)
    prev_pos = waypoints[max(0, prev_pos_ind)]

    next_pos_ind = prev_pos_ind + 1
    next_pos = waypoints[min(len(waypoints)-1, next_pos_ind)]

    a_vec = next_pos - prev_pos  # vector from last wp to next wp
    # Angle at which route is tilted from y
    omega = atan2(a_vec[0], a_vec[1])
    rot = [[cos(-omega), sin(-omega)],
           [-sin(-omega), cos(-omega)]]

    # Relative positon w.r.t last waypoint
    rel_pos = (current_pos[0] - prev_pos[0], current_pos[1] - prev_pos[1])

    # Rotate position vector by the same angle
    pos_rot = dot_Matrix_Vector(rot, rel_pos)

    # After rotation, "x" component is the CTE
    # Offset = +ve for left of route
    cte = -pos_rot[0]

    return cte


def compute_curvature(a, b, c):
    CA = sqrt((c[1] - a[1])**2 + (c[0] - a[0])**2)
    ang_abc = get_angle(a, b, c)
    CA = max(CA, 0.01)  # Avoid division by zero
    curvature = (2 * sin(ang_abc)) / CA
    return curvature


def has_high_curvature(points, K_THRESH):

    max_cur = 0
    for i in range(len(points)-2):
        a = points[i]
        b = points[i+1]
        c = points[i+2]
        cur = abs(compute_curvature(a,b,c))
        if cur >= K_THRESH:
            return True

        if cur > max_cur:
            max_cur = cur

    return False


# i can be 1/-1
def get_point_until_d(points_x, points_y, i, inc, max_dist):
    points_ahead = []
    d = 0
    k = i
    while d < max_dist and k < len(points_x)-1 and k > 0:
        d_int = int(d)
        a = [points_x[k], points_y[k]]
        b = [points_x[k+inc],points_y[k+inc]]

        d += euc(a, b)
        k+=inc
        ah = [points_x[k], points_y[k], 0.0]
        points_ahead.append(ah)

    # Add heading info
    if len(points_ahead) > 1:
        for k in range(len(points_ahead)-1):
            a = points_ahead[k]
            b = points_ahead[k+1]
            h0 = get_heading(a,b)
            points_ahead[k] = [a[0], a[1], h0]

        #print("points_ahead:", points_ahead)
        points_ahead[-1] = [points_ahead[-1][0], points_ahead[-1][1], points_ahead[-2][2]]


    return points_ahead

def closest_node(node, nodes):
    #nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)


def closest_node_behind(node, nodes, current_heading):
    dist_2 = np.sum((nodes - node) ** 2, axis=1)
    min_ind = np.argmin(dist_2)
    inds = [min_ind, min(min_ind + 1, len(nodes)-1), max(0, min_ind - 1)]

    min_angle = 0.
    for (ind) in inds:
        diff = abs(angle_diff(current_heading, get_heading(node, nodes[min_ind])))
        if diff <= min_angle:
            min_ind = ind
            min_angle = diff

    return min_ind


def angle_diff(h1, h2):
    return math.atan2(math.sin(h1-h2), math.cos(h1-h2))

def get_rows(file_name):
    try:
        rows = []

        with open(file_name, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                rows.append([float(row[-2]), float(row[-1]), 0.0])

        return rows
    except Exception as e:
        return None


def is_close_to_curve(builder, data, i, LOOK_AHEAD_METERS=10., LOOK_BEHIND_METERS=5.):

    np_data = np.array(np.copy(data))

    #print("np_data:", np_data)
    points_x = list(np_data[:, 0])
    #print("points_x:", points_x)
    points_y = list(np_data[:, 1])
    #print("points_y:", points_y)

    points_ahead = get_point_until_d(points_x=points_x, points_y=points_y, i=i, inc=1, max_dist=LOOK_AHEAD_METERS)
    curve_ahead = has_high_curvature(points_ahead, K_THRESH)

    points_behind = get_point_until_d(points_x=points_x, points_y=points_y, i=i, inc=-1, max_dist=LOOK_BEHIND_METERS)
    curve_behind = has_high_curvature(points_behind, K_THRESH)

    if builder is not None:
        builder.primitive('/actual_path_ahead')\
            .polyline(np.array(points_ahead).flatten())\
            .id('actual_path_ahead')

        builder.primitive('/actual_path_behind')\
            .polyline(np.array(points_behind).flatten())\
            .id('actual_path_behind')

    if curve_ahead:
        return curve_ahead

    return curve_behind

def get_actual_path(data, i, machine_width, cte, LOOK_AHEAD_METERS=10.):
    W_half = machine_width / 2.0
    cte_half = (cte / 2.0)

    for j in range(len(data)-1):
        row0 = data[i]
        row1 = data[i+1]
        h0 = get_heading(row0, row1)
        row0[2] = h0

    np_data = np.array(data)
    points_x = list(np_data[:, 0])
    points_y = list(np_data[:, 1])

    path = get_point_until_d(points_x=points_x, points_y=points_y, i=i, inc=1, max_dist=LOOK_AHEAD_METERS)

    path = np.array(path)


    #print("self.path:", path)

    cte_offset_x = (cte_half * np.cos(path[:, 2] + pi / 2))
    cte_offset_y = (cte_half * np.sin(path[:, 2] + pi / 2))

    left_offset_x = (W_half * np.cos(path[:, 2] + pi / 2)) + cte_offset_x
    left_offset_y = (W_half * np.sin(path[:, 2] + pi / 2)) + cte_offset_y

    right_offset_x = (W_half * np.cos(path[:, 2] - pi / 2)) + cte_offset_x
    right_offset_y = (W_half * np.sin(path[:, 2] - pi / 2)) + cte_offset_y

    left = np.column_stack([path[:, 0] + left_offset_x,
                            path[:, 1] + left_offset_y])
    right = np.column_stack([path[:, 0] + right_offset_x,
                            path[:, 1] + right_offset_y])

    return path, left, right


def get_actual_path_poly(data, i, machine_width, cte, LOOK_AHEAD_METERS=10.):

    #print("Using actual path poly")
    _path, left, right = get_actual_path(
        data=data,
        i=i,
        cte=cte,
        machine_width=machine_width,
        LOOK_AHEAD_METERS=LOOK_AHEAD_METERS)

    z = 1.1
    left = np.column_stack((
        left,
        np.full(left.shape[0], z)
    ))
    right = np.column_stack((
        np.flipud(right),
        np.full(right.shape[0], z)
    ))

    return np.concatenate((
        left.flatten(),
        right.flatten(),
    )).tolist()