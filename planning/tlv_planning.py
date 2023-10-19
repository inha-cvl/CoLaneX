import rospy
import sys
import signal
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

from libs.map import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
import libs.planner_utils as p
from libs.rviz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

def signal_handler(sig, frame):
    sys.exit(0)

class SafeDistance:
    def __init__(self):
        self.state = 'WAIT'
        self.base_lla = [35.64588122580907,128.40214778762413, 46.746] #KIAPI
        #self.base_lla = [37.383378,126.656798, 46.746] #SONGDO-SITE
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
        self.tlv_path = []
        self.tlv_geojson = None
        self.hlv_path = None
        self.hlv_v = None
        self.hlv_signal = 0 #0 : none, 1 : left, 2 : right
        self.safety = 0 #0: none, 1: safe, 2: dangerous

        self.path_make_cnt = 0
        self.final_path = None

        self.calc_safe_cnt = 0
        self.ego_v = 12 #m/s -> callback velocity
        self.x_p = 1.5
        self.x_c = 200
        self.intersection_radius = 1.0
        self.d_c = 15 # If at noraml road || on high way == 0

        self.check_safety = []
        self.confirm_safety = False

        rospy.Subscriber('/car/tlv_pose', Pose, self.tlv_pose_cb)
        rospy.Subscriber('/v2x/hlv_pose', Pose, self.hlv_pose_cb)
        rospy.Subscriber('/v2x/hlv_path', Marker, self.hlv_path_cb)
        rospy.Subscriber('/tlv_signal', Int8, self.tlv_signal_cb)
        ####################

        self.pub_lanelet_map = rospy.Publisher('/planning/lanelet_map', MarkerArray, queue_size = 1, latch=True)
        self.pub_tlv_path = rospy.Publisher('/planning/tlv_path', Marker, queue_size=1)
        self.pub_intersection = rospy.Publisher('/planning/intersection', Marker, queue_size=1)
        self.pub_move = rospy.Publisher('/planning/move', Marker, queue_size=1)
        self.pub_safety = rospy.Publisher('/planning/safety', Int8, queue_size=1)
        self.pub_tlv_geojson = rospy.Publisher('/planning/tlv_geojson', String, queue_size=1)
        self.pub_hlv_geojson = rospy.Publisher('/planning/hlv_geojson', String, queue_size=1)
        self.pub_tlv_state = rospy.Publisher('/tlv_state', Int8, queue_size=1)
        
        lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)
        p.lanelets = self.lmap.lanelets

    def tlv_pose_cb(self, msg): 
        self.ego_pos = p.convert2enu(self.base_lla, msg.position.x, msg.position.y)
        self.ego_v = msg.orientation.x
    
    def hlv_pose_cb(self, msg):
        self.hlv_v = msg.orientation.x
        if msg.orientation.y == 1 or msg.orientation.y == 2:
            self.hlv_signal = msg.orientation.y

    def hlv_path_cb(self, msg):
        hlv_path = [( pt.x, pt.y) for pt in msg.points]
        if len(hlv_path) > 0:
            self.hlv_path = p.ref_interpolate(hlv_path, self.precision)[0]
        # compress_path = do_compressing(self.hlv_path, 10)
        if len(hlv_path) > 0:
            hlv_geojson = p.to_geojson(hlv_path, self.base_lla)
            self.pub_hlv_geojson.publish(hlv_geojson)
    
    def tlv_signal_cb(self, msg):
        if msg.data != 0:
            self.check_safety = []
            self.confirm_safety = False
            self.hlv_signal = 0
    
    def publish_state(self):
        state = 0
        if self.hlv_signal == 1:
            state = 1
        elif self.hlv_signal == 2:
            state = 2
        if self.safety == 1:
            state = 3
        elif self.safety == 2:
            state = 4
            
        self.pub_tlv_state.publish(Int8(state))

    def need_update(self):
        if self.final_path == None:
            return 0
        threshold = ((self.ego_v * MPS_TO_KPH)*self.M_TO_IDX) * 1.5
        idx = p.find_nearest_idx(self.final_path, self.ego_pos)
        if len(self.final_path) - idx <= threshold:
            return 1
        else:
            return -1

    def get_node_path(self):
        if self.ego_pos == None:
            return
        
        final_path = []
        compress_path = []
        need_update = self.need_update()
        if need_update != -1:
            r0 = []
            if need_update == 0:
                start_pose = self.ego_pos
            elif need_update == 1:
                start_pose = self.final_path[-1]
                idx = p.find_nearest_idx(self.final_path, self.ego_pos)
                r0 = self.final_path[idx:]

            ego_lanelets = p.lanelet_matching(self.tmap.tiles, self.tmap.tile_size, start_pose)
            if ego_lanelets == None:
                return 
            x1 = self.ego_v * MPS_TO_KPH + self.ego_v * self.x_p + self.x_c #m
            r1, _, _ = p.get_straight_path(ego_lanelets[0], ego_lanelets[1], x1)

            final_path = r0+r1
            #compress_path = do_compressing(final_path, 10)
            self.final_path = p.ref_interpolate(final_path, self.precision)[0]
            # cause limitation of v2x max length 
            self.tlv_path = p.limit_path_length(self.final_path, 50)
            self.tlv_geojson = p.to_geojson(self.tlv_path, self.base_lla)
            
            
        tlv_path_viz = TLVPathViz(self.tlv_path)
        self.pub_tlv_path.publish(tlv_path_viz)
        self.pub_tlv_geojson.publish(self.tlv_geojson)
    
    def is_insied_circle(self, pt1, pt2, radius):
        distance = math.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)
        if distance <=  radius:
            return True
        else:
            return False

    def calc_safe_distance(self):
        if self.hlv_path == None or self.hlv_v == None or self.final_path == None:
            return
        
        find = False
        inter_pt = None
        inter_idx = 0
        hlv_idx = 0

        for hi, hwp in enumerate(self.hlv_path):
            if find:
                break
            for ti, twp in enumerate(self.final_path):
                if self.is_insied_circle(twp, hwp, self.intersection_radius):
                    inter_pt = twp
                    inter_idx = ti
                    hlv_idx = hi
                    find = True
                    break

        if find and not self.confirm_safety and self.hlv_signal != 0:
            self.pub_intersection.publish(Sphere('intersection', 0, inter_pt, 5.0, (33/255, 255/255, 144/255, 0.7)))
            now_idx = p.find_nearest_idx(self.final_path, self.ego_pos)
            d1 = (inter_idx-now_idx)*self.IDX_TO_M
            d2 = self.ego_v * ((hlv_idx*self.IDX_TO_M)/self.hlv_v) if self.hlv_v != 0 else 0
            d2_idx = int(d2*self.M_TO_IDX) + now_idx 
            dg = d1-d2
            ds = self.ego_v*3.6-self.d_c #m
                      
            if inter_idx <= now_idx+10:
                safety = 0  
            else:
                safety = 1 if dg > ds else 2 # 1 : Safe, 2 : Dangerous
            
            if self.safety != safety and len(self.check_safety) == 0:
                self.check_safety.append(safety)
                self.safety = safety
            elif self.safety == safety and len(self.check_safety) > 0:
                self.check_safety.append(safety)
            elif self.safety != safety and len(self.check_safety) > 0:
                self.confirm_safety = True
            elif self.safety == safety and len(self.check_safety) == 4:
                self.confirm_safety = True
                

            if self.safety == 1:
                self.pub_intersection.publish(Sphere('intersection', 0, inter_pt, 5.0, (33/255, 255/255, 144/255, 0.7)))
            elif self.safety == 2:
                self.pub_intersection.publish(Sphere('intersection', 0, inter_pt, 5.0, (255/255, 38/255, 85/255, 0.7)))
            if d2_idx < len(self.final_path):  
                self.pub_move.publish(Sphere('move', 0, self.final_path[d2_idx], 3.0, (91/255, 113/255, 255/255, 0.7)))
        else:
            self.safety = 0

    def run(self):
        self.state = 'RUN'
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.publish_state()
            self.get_node_path()
            self.calc_safe_distance()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('SafeDistance', anonymous=False)
    sd = SafeDistance()
    sd.run()

if __name__ == "__main__":
    main()
