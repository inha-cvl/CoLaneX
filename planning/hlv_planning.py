import rospy
import sys
import signal

from std_msgs.msg import Int8, String
from geometry_msgs.msg import  Pose
from visualization_msgs.msg import Marker, MarkerArray

from libs.map import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
import libs.planner_utils as p
from libs.rviz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

def signal_handler(sig, frame):
    sys.exit(0)

class DynamicPath:
    def __init__(self):
        self.base_lla = [35.64588122580907,128.40214778762413, 46.746] #KIAPI
        #self.base_lla = [37.383378,126.656798, 7] #SONGDO-SITE
        self.tile_size = 5.0
        self.cut_dist = 15.0
        self.precision = 0.5

        #self.lmap = LaneletMap("songdo-site.json")
        self.lmap = LaneletMap("KIAPI.json")
        self.tmap = TileMap(self.lmap.lanelets, self.tile_size)
        self.graph = MicroLaneletGraph(self.lmap, self.cut_dist).graph
        self.M_TO_IDX = 1/self.precision
        self.IDX_TO_M = self.precision
        self.ego_pos = None
        self.hlv_path = []
        self.hlv_geojson = None


        self.path_make_cnt = 0
        self.ego_v = 12 #m/s -> callback velocity
        self.signal = 0 # 0 : default, 1 : left, 2 : right
        self.temp_signal = self.signal
        self.x_p = 3
        self.x_c = 50
        self.x2_i = 30
        self.x2_v_th = 27
        self.merging_point = None
        self.hlv_merged = 0
        self.x3_c = 7
        self.final_path = None
        self.tlv_signal = 0 # 0 : none, 1 : ok, 2 : no

        rospy.Subscriber('/car/hlv_pose', Pose, self.hlv_pose_cb)
        rospy.Subscriber('/v2x/tlv_path', Marker, self.tlv_path_cb)
        rospy.Subscriber('/v2x/tlv_pose', Pose, self.tlv_pose_cb)
        rospy.Subscriber('/hlv_signal', Int8, self.hlv_signal_cb)
        self.pub_lanelet_map = rospy.Publisher('/planning/lanelet_map', MarkerArray, queue_size = 1, latch=True)
        self.pub_hlv_path = rospy.Publisher('/planning/hlv_path', Marker, queue_size=1)
        self.pub_hlv_ipath = rospy.Publisher('/planning/hlv_ipath', Marker, queue_size=1)
        self.pub_hlv_state = rospy.Publisher('/hlv_state', Int8, queue_size=1)
        self.pub_hlv_geojson = rospy.Publisher('/planning/hlv_geojson', String, queue_size=1)
        self.pub_tlv_geojson = rospy.Publisher('/planning/tlv_geojson', String, queue_size=1)
        self.pub_hlv_merged = rospy.Publisher('/planning/hlv_merged', Int8, queue_size=1)
        
        lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)
        p.lanelets = self.lmap.lanelets

    def hlv_pose_cb(self, msg):
        self.ego_pos = p.convert2enu(self.base_lla, msg.position.x, msg.position.y)
        self.ego_v = msg.orientation.x

    def tlv_pose_cb(self, msg):
        self.tlv_signal = msg.orientation.y

    def tlv_path_cb(self,msg):
        tlv_path = [(pt.x, pt.y) for pt in msg.points]
        if len(tlv_path) > 0:
            tlv_geojson = p.to_geojson(tlv_path, self.base_lla)
            self.pub_tlv_geojson.publish(tlv_geojson)

    def hlv_signal_cb(self, msg):
        if msg.data == 4:
            self.hlv_merged = 0
        self.signal = msg.data

    def publish_state(self):
        state = 0
        if self.signal == 1:
            state = 1
        elif self.signal == 2:
            state = 2
        if self.tlv_signal == 1:
            state = 3
        elif self.tlv_signal == 2:
            state = 4
        self.pub_hlv_state.publish(Int8(state))

    def need_update(self):
        if self.final_path == None:
            return 0
        if self.temp_signal != self.signal and self.signal != 0:
            self.temp_signal = self.signal
            return 2
        threshold = ((self.ego_v * MPS_TO_KPH)*self.M_TO_IDX) * 2.5
        idx = p.find_nearest_idx(self.final_path, self.ego_pos)

        if len(self.final_path) - idx <= threshold:
            return 1
        else:
            return -1

    def get_change_path(self, s_n, s_i,  path_len, to=1):
        wps, u_n, u_i = p.get_straight_path( s_n, s_i, path_len)
        c_pt = wps[-1]
        l_id, r_id = p.get_neighbor( u_n)
        n_id = l_id if to==1 else r_id

        if n_id != None:
            r = self.lmap.lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = p.find_nearest_idx(r, c_pt)
        else:
            r = wps
        return r, u_n, u_i
    

    def make_path(self, update_type):
        r0 = []
        c = 0
        if update_type == 0 or update_type == 2:
            start_pose = self.ego_pos
        else:
            start_pose = self.final_path[-1]
            idx = p.find_nearest_idx(self.final_path, self.ego_pos)
            r0 = self.final_path[idx:]
            c = self.x_c
        
       
        ego_lanelets = p.lanelet_matching(self.tmap.tiles, self.tmap.tile_size, start_pose)
        if ego_lanelets == None:
            return None
        
        if ego_lanelets == None:
            return None
        
        x1 = self.ego_v * MPS_TO_KPH + c if self.ego_v > 0.5 else 50
        r1, n1, i1 = p.get_straight_path(ego_lanelets[0], ego_lanelets[1], x1)
        if self.signal == 0 or self.signal == 3:
            final_path = r0+r1
        else:

            x2 = self.x2_i if self.ego_v < self.x2_v_th else self.ego_v * self.x_p
            _, n2, i2 = self.get_change_path(n1, i1, x2, self.signal)
            self.merging_point = n2
            x3 = self.x3_c + self.ego_v * self.x_p
            r3, _, _ = p.get_straight_path( n2, i2, x3)
            final_path = r0+r1+r3

        return final_path 

    def check_is_merged(self):
        now_lanelets = p.lanelet_matching(self.tmap.tiles, self.tmap.tile_size, self.ego_pos)
        if now_lanelets[0] == self.merging_point:
            self.hlv_merged = 1
        self.pub_hlv_merged.publish(Int8(self.hlv_merged))

    def get_node_path(self):
        if self.ego_pos == None:
            return
        
        final_path = []
        need_update = self.need_update()
        if need_update != -1:
            final_path = self.make_path(need_update)
            if final_path == None or len(final_path) <= 0:
                return

            self.final_path = p.smooth_interpolate(final_path, self.precision)
            self.hlv_path = p.limit_path_length(self.final_path, 50) 
            self.hlv_geojson = p.to_geojson(self.hlv_path, self.base_lla)
                    
        hlv_path_viz = HLVPathViz(self.hlv_path)
        final_path_viz = HLVPathViz(self.final_path)
        self.pub_hlv_ipath.publish(final_path_viz)
        self.pub_hlv_path.publish(hlv_path_viz)
        self.pub_hlv_geojson.publish(self.hlv_geojson)
        

    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.publish_state()
            self.get_node_path()
            self.check_is_merged()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('DynamicPath', anonymous=False)
    dp = DynamicPath()
    dp.run()

if __name__ == "__main__":
    main()
