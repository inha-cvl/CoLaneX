import geopandas as gpd
import matplotlib.pyplot as plt
import utm
import pymap3d as pm
import numpy as np
import json

base_lla = (37.2292221592864, 126.76912499027308, 29.18400001525879)
rotation_angle = 88.35


def rotate_points(x, y, angle):
    angle_rad = np.radians(angle)  
    x = np.array(x)  
    y = np.array(y)  
    x_rotated = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rotated = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rotated, y_rotated


def to_cartesian( tx, ty, alt=None):
    lat, lon = utm.to_latlon(tx, ty, 52, 'N')
    if alt is None:
        x, y, _ = pm.geodetic2enu(lat, lon, base_lla[2], base_lla[0], base_lla[1], base_lla[2])
        return x, y
    else:
        x, y, z = pm.geodetic2enu(lat, lon, alt, base_lla[0], base_lla[1], base_lla[2])
        return x, y, z
        
def process_surface_line():
    shp_file = './KCity-Cut./KCity-Cut.shp'.format(map)
    gdf = gpd.read_file(shp_file)
    lines_x, lines_y = [], []
    for index, row in gdf.iterrows():
        if row['geometry'].geom_type == 'LineString':
            line_x, line_y = [], []
            for point in row['geometry'].coords:
                lon, lat = point[0], point[1]
                x, y = to_cartesian(lon, lat)
                line_x.append(x)
                line_y.append(y)
            rotated_x, rotated_y = rotate_points(np.array(line_x), np.array(line_y), rotation_angle)
            lines_x.append(rotated_x)
            lines_y.append(rotated_y)
    surface_lines = (lines_x, lines_y)
    return surface_lines


plt.figure(figsize=(20, 3))

surface_lines = process_surface_line()
for line_x, line_y in zip(*surface_lines):
    plt.plot(line_x, line_y, color='black', linewidth=0.5)

s = 's5'
with open(f'../log/v2x/{s}.json', 'r') as file:
    data = json.load(file)

def get_enu(lat,lon):
    lla = pm.geodetic2enu(lat, lon, 0, base_lla[0], base_lla[1], base_lla[2])
    return lla[0], lla[1]

# Extracting coordinates for plotting
rotated_points = rotate_points(
    *np.array([(get_enu(lat, lon)) for lat, lon, _ in data['ego'] if lat != 0 and lon != 0]).T, 
    rotation_angle
)
ego_path = [rotated_points[0].tolist(), rotated_points[1].tolist()]

rotated_points = rotate_points(
    *np.array([(get_enu(lat, lon)) for lat, lon, _ in data['target'] if lat != 0 and lon != 0]).T, 
    rotation_angle
)
v2x_path = [rotated_points[0].tolist(), rotated_points[1].tolist()]

with open(f'../log/lidar/{s}.json', 'r') as file:
    data = json.load(file)

rotated_points = rotate_points(
    *np.array([(get_enu(lat, lon)) for lat, lon, _ in data['target'] if lat != 0 and lon != 0]).T, 
    rotation_angle
)
lidar_path = [rotated_points[0].tolist(), rotated_points[1].tolist()]

# Plotting

plt.plot(ego_path[0], ego_path[1], 'r-', label='Ego Path')  # latitude on y-axis, longitude on x-axis
plt.plot(v2x_path[0], v2x_path[1], 'b-', label='V2X Path')  # latitude on y-axis, longitude on x-axis
plt.plot(lidar_path[0], lidar_path[1], 'g-', label='LiDAR Path')  # latitude on y-axis, longitude on 

plt.grid(False)
plt.xticks([])
plt.yticks([])
plt.show()