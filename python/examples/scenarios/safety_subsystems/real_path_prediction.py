import csv
from math import pi
import numpy as np
from math import cos, asin, sqrt, pi,atan2, sin
import math

K_THRESH = .1

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
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

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

    np_data = np.array(data)

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

def get_actual_path(data, i, machine_width, LOOK_AHEAD_METERS=10.):
    W_half = machine_width / 2.0

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

    left = np.column_stack([path[:, 0] + W_half * np.cos(path[:, 2] + pi / 2),
                            path[:, 1] + W_half * np.sin(path[:, 2] + pi / 2)])
    right = np.column_stack([path[:, 0] + W_half * np.cos(path[:, 2] - pi / 2),
                            path[:, 1] + W_half * np.sin(path[:, 2] - pi / 2)])

    return path, left, right


def get_actual_path_poly(data, i, machine_width, LOOK_AHEAD_METERS=10.):

    #print("Using actual path poly")
    _path, left, right = get_actual_path(
        data=data,
        i=i,
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