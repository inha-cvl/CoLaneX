from math import radians, sin, cos, sqrt, atan2

def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0 
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance

def calculate_ttc(ego, target):
    ego_lat, ego_lng, ego_v = ego
    target_lat, target_lng, target_v = target
    current_distance = haversine(ego_lat, ego_lng, target_lat, target_lng) * 1000
    current_relative_speed = target_v - ego_v
    
    if current_relative_speed != 0:
        ttc = abs(current_distance / current_relative_speed)
        return ttc
    else:
        return float('inf')
