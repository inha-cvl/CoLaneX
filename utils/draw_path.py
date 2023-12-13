import matplotlib.pyplot as plt
import json
import pymap3d as pm

s = 's7'
with open(f'./log/v2x/{s}.json', 'r') as file:
    data = json.load(file)

base_lla = [37.2292221592864, 126.76912499027308, 29.18400001525879]

def get_enu(lat,lon):
    lla = pm.geodetic2enu(lat, lon, 0, base_lla[0], base_lla[1], base_lla[2])
    return lla[0], lla[1]

# Extracting coordinates for plotting
ego_path = list(zip(*[(get_enu(lat, lon)) for lat, lon, _ in data['ego'] if lat != 0 and lon != 0]))


v2x_path = list(zip(*[(get_enu(lat, lon)) for lat, lon, _ in data['target'] if lat != 0 and lon != 0]))


with open(f'./log/lidar/{s}-2.json', 'r') as file:
    data = json.load(file)

lidar_path = list(zip(*[(get_enu(lat, lon)) for lat, lon, _ in data['target'] if lat != 0 and lon != 0]))

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(ego_path[0], ego_path[1], 'r-', label='Ego Path')  # latitude on y-axis, longitude on x-axis
plt.plot(v2x_path[0], v2x_path[1], 'b-', label='V2X Path')  # latitude on y-axis, longitude on x-axis
plt.plot(lidar_path[0], lidar_path[1], 'g-', label='LiDAR Path')  # latitude on y-axis, longitude on x-axis


plt.axis('off')
plt.xticks([])
plt.yticks([])

# Save the plot with a transparent background
plt.savefig(f'./png/{s}-2.png', transparent=True)