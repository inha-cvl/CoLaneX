import rospy
import sys
import time
import signal
import pymap3d
import json
from scipy.spatial import KDTree
from std_msgs.msg import Int8, Float32, String
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker
from novatel_oem7_msgs.msg import INSPVA

from libs.map import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
from libs.planner_utils import *
from libs.rviz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

def signal_handler(sig, frame):
    sys.exit(0)

class DynamicPath:
    def __init__(self):
        self.state = 'WAIT'
        self.base_lla = [35.64588122580907,128.40214778762413, 46.746]
        self.tile_size = 5.0
        self.cut_dist = 15.0
        self.precision = 0.5

        self.lmap = LaneletMap("KIAPI.json")
        self.tmap = TileMap(self.lmap.lanelets, self.tile_size)
        self.graph = MicroLaneletGraph(self.lmap, self.cut_dist).graph
        self.M_TO_IDX = 1/self.precision
        self.IDX_TO_M = self.precision
        self.ego_pos = None

        self.path_make_cnt = 0
        self.ego_v = 12 #m/s -> callback velocity
        self.signal = 1 # 0 : default, 1 : left, 2 : right
        self.x_p = 1.5
        self.x2_i = 10
        self.x3_c = 7


        rospy.Subscriber('/car/hlv_pose', Pose, self.hlv_pose_cb)
        self.pub_lanelet_map = rospy.Publisher('/planning/lanelet_map', MarkerArray, queue_size = 1, latch=True)
        self.pub_hlv_path = rospy.Publisher('/planning/hlv_path', Marker, queue_size=1)
        self.pub_hlv_geojson = rospy.Publisher('/planning/hlv_geojson', String, queue_size=1)
        
        lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)


    def hlv_pose_cb(self, msg):
        self.ego_pos = convert2enu(self.base_lla, msg.position.x, msg.position.y)
        self.ego_v = msg.orientation.x

    def get_node_path(self):
        if self.ego_pos == None:
            return
        st = time.time()
        ego_lanelets = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, self.ego_pos)
        ego_node = node_matching(self.lmap.lanelets, ego_lanelets[0], ego_lanelets[1])
        ego_waypoints = self.lmap.lanelets[ego_lanelets[0]]['waypoints']
        ego_lanelets_len = len(ego_waypoints)
        x1 = self.ego_v * MPS_TO_KPH + self.ego_v * self.x_p #m
        x1_idx = ego_lanelets[1]+int(x1 * self.M_TO_IDX)
        
        x1_ego_not_same = False
        if x1_idx >= ego_lanelets_len:
            x1_ego_not_same = True
            x1_s_node = self.lmap.lanelets[ego_lanelets[0]]['successor'][0]
            x1_idx = (x1_idx-ego_lanelets_len)+1
        else:
            x1_s_node = ego_lanelets[0]
        n1 = node_matching(self.lmap.lanelets, x1_s_node, x1_idx)
        n1_waypoints = self.lmap.lanelets[x1_s_node]['waypoints']
        if x1_ego_not_same:
            r1 = ego_waypoints[ego_lanelets[1]:]+n1_waypoints[:x1_idx+1]
        else:
            r1 = n1_waypoints[ego_lanelets[1]:x1_idx+1]

        x1_lanelets_len = len(n1_waypoints)
        x2 = self.x2_i + self.ego_v * self.x_p
        x2_idx = x1_idx+int(x2*self.M_TO_IDX)

        if x2_idx >= x1_lanelets_len:
            x2_s_node = self.lmap.lanelets[x1_s_node]['successor'][0]
            x2_idx = (x2_idx-x1_lanelets_len)+1
        else:
            x2_s_node = x1_s_node
        n2 = node_matching(self.lmap.lanelets, x2_s_node, x2_idx)
        n2_waypoints = self.lmap.lanelets[x2_s_node]['waypoints']
        x2_pt = n2_waypoints[x2_idx]

        if self.signal == 1 :
            n2_neighbor = self.lmap.lanelets[x2_s_node]['adjacentLeft']
        elif self.signal == 2 :
            n2_neighbor = self.lmap.lanelets[x2_s_node]['adjacentRight']

        if n2_neighbor != None:
            n2_waypoints = self.lmap.lanelets[n2_neighbor]['waypoints']
            x2_s_node = n2_neighbor
        else:
            return
        
        x2_idx = find_nearest_idx(n2_waypoints, x2_pt)
        n2 = node_matching(self.lmap.lanelets, x2_s_node, x2_idx)
        n2_waypoints = self.lmap.lanelets[x2_s_node]['waypoints']

        x2_lanelets_len = len(n2_waypoints)
        x3 = self.x3_c + self.ego_v * self.x_p
        x3_idx = x2_idx + int(x3*self.M_TO_IDX)

        x3_x2_not_same = False
        if x3_idx >= x2_lanelets_len:
            x3_x2_not_same = True
            x3_s_node = self.lmap.lanelets[x2_s_node]['successor'][0]
            x3_idx = (x3_idx-x2_lanelets_len)+1
        else:
            x3_s_node = x2_s_node
        n3 = node_matching(self.lmap.lanelets, x3_s_node, x3_idx)
        n3_waypoints = self.lmap.lanelets[x3_s_node]['waypoints']
        if x3_x2_not_same:
            r3 = n2_waypoints[x2_idx:]+n3_waypoints[:x3_idx+1]
        else:
            r3 = n3_waypoints[x2_idx:x3_idx+1]

        self.hlv_path = ref_interpolate(r1 + r3, self.precision)[0]
        compress_path = [r1[0], r1[-1], r3[0], r3[-1]]
        hlv_path_viz = HLVPathViz(self.hlv_path)
        self.pub_hlv_path.publish(hlv_path_viz)
        
        mt = format((time.time()-st)*1000, ".3f")
        print(f"Ego Node Matching : {mt}ms , Nodes [{ego_node}]-[{n1}]-[{n2}]-[{n3}]")

        # if self.path_make_cnt > 20:
        #     self.state = 'Path'

        #     latlng_waypoints = []
        #     for wp in compress_path:
        #         lat, lng, _ = pm.enu2geodetic(wp[0], wp[1], 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        #         latlng_waypoints.append((lng, lat))

        #     feature = {
        #         "type":"Feature",
        #         "geometry":{
        #             "type":"MultiLineString",
        #             "coordinates":[latlng_waypoints]
        #         },
        #         "properties":{}
        #     }
        #     self.hlv_geojson = json.dumps(feature)
        #     with open('./path/hlv.json', 'w') as file:
        #         json.dump(feature, file)

        # else:
        #     self.path_make_cnt += 1

    def run(self):
        self.state = 'RUN'
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.state != 'Path':
                self.get_node_path()
            if self.state == 'Path':
                self.pub_hlv_geojson.publish(self.hlv_geojson)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('DynamicPath', anonymous=False)
    dp = DynamicPath()
    dp.run()

if __name__ == "__main__":
    main()
