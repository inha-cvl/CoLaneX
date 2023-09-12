
import numpy as np
import pymap3d as pm
import json
from libs.quadratic_spline_interpolate import QuadraticSplineInterpolate

M_TO_IDX = 1/0.5
IDX_TO_M = 0.5

def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)

def convert2enu(base, lat, lng):
    x, y, _ = pm.geodetic2enu(lat, lng, 20, base[0], base[1], base[2])
    return [x, y]

def to_geojson(path, base):

    latlng_waypoints = []
    for wp in path:
        lat, lng, _ = pm.enu2geodetic(wp[0], wp[1], 0, base[0], base[1], base[2])
        latlng_waypoints.append((lng, lat))

    feature = {
        "type":"Feature",
        "geometry":{
            "type":"MultiLineString",
            "coordinates":[latlng_waypoints]
        },
        "properties":{}
    }   
    return json.dumps(feature)

def lanelet_matching(tile, tile_size, t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tile.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            min_dist = dist
                            l_id = id_
                            l_idx = data['idx'][idx]
    if l_id is not None:
        return (l_id, l_idx)
    else:
        return None

def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points

def get_neighbor(lanelet, node):
    l_id = lanelet[node]['adjacentLeft']
    r_id = lanelet[node]['adjacentRight']
    return l_id, r_id

def get_whole_neighbor(lanelet, node):
    num = 1
    find_node = node
    left_most = True
    right_most = True
    left_lanes = []
    right_lanes = []

    while left_most:
        if lanelet[find_node]['adjacentLeft'] != None:
            find_node = lanelet[find_node]['adjacentLeft']
            left_lanes.append(find_node)
            num += 1
            
        else:
            left_most = False
            find_node = node
    
    while right_most:
        if lanelet[find_node]['adjacentRight'] != None:
            find_node = lanelet[find_node]['adjacentRight']
            right_lanes.append(find_node)
            num += 1
            
        else:
            right_most = False
            find_node = node

    me_idx = len(left_lanes)

    return left_lanes, right_lanes, me_idx


    
def find_most_successor(lanelet, check_l):
    most_successor = None
    for c in check_l:
        if len(lanelet[c]['successor']) <= 0:
            continue
        else:
            most_successor = lanelet[c]['successor'][0]
            break
    return most_successor

def get_possible_successor(lanelet, node, prior='Left'):
    successor = None
    left_lanes, right_lanes, me = get_whole_neighbor(lanelet, node)
    if len(lanelet[node]['successor']) <= 0:
        if prior == 'Left':
            check_a = left_lanes
            check_b = right_lanes
        else:   
            check_a = right_lanes
            check_b = left_lanes
        
        most_successor = find_most_successor(lanelet, check_a)
        if most_successor == None:
            most_successor = find_most_successor(lanelet, check_b)

        successor = most_successor
    else:
        successor = lanelet[node]['successor'][0]

    return successor

def get_straight_path(lanelet, s_n, s_i, path_len):
    wps = lanelet[s_n]['waypoints']
    lls_len = len(wps)

    u_n = s_n
    u_i = s_i+int(path_len*M_TO_IDX)
    e_i = u_i

    while u_i >= lls_len:
        _u_n = get_possible_successor(lanelet, u_n, prior='Left')
        if _u_n == None:
            e_i = len(wps-1)
            break
        u_n = _u_n
        u_i -= lls_len
        e_i += u_i
        u_wp = lanelet[u_n]['waypoints']
        lls_len = len(u_wp)
        wps += u_wp
    r = wps[s_i:e_i]

    return r, u_n, u_i

def ref_interpolate(points, precision):
    points = filter_same_points(points)
    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    itp_points = []
    for ds in np.arange(0.0, itp.s[-1], precision):
        x, y = itp.calc_position(ds)
        itp_points.append((float(x), float(y)))

    return itp_points, itp.s[-1]

def node_matching(lanelet, l_id, l_idx):
    node_id = l_id
    if lanelet[l_id].get('cut_idx') is not None:
        for n, (s_idx, e_idx) in enumerate(lanelet[l_id]['cut_idx']):
            if l_idx >= s_idx and l_idx < e_idx:
                node_id += '_%s' % (n)
                break
    
    return node_id

def do_compressing(points, n):
    if len(points) < 2:
        return 

    x1, y1 = points[0]
    x2, y2 = points[-1]

    new_points = [(x1, y1)]
    for i in range(n, len(points), n):
        x, y = points[i]
        new_points.append((x, y))

    new_points.append((x2, y2))
    return new_points


def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx


def limit_path_length(path, max_length):
    if len(path) <= max_length:
        return path
    
    new_path = [path[0]]
    interval = (len(path)-1)/(max_length-2)

    for i in range(1, max_length -1):
        index = int(i*interval)
        new_path.append(path[index])
    
    new_path.append(path[-1])
    return new_path