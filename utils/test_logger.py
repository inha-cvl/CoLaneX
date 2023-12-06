import sys
import time
import json
import rospy
from geometry_msgs.msg import Pose, PoseArray, Vector3
from std_msgs.msg import Int8

from ttc_calculator import calculate_ttc

class TestLogger():
    def __init__(self, file_name, test_mode):
        
        self.test_mode = test_mode
        self.state = 'wait'
        self.path = f"./log/{file_name}.json"
        self.passthorugh_start = 0
        self.log_data = {}

        self.hlv_merged = 0
        self.ego = [0,0,0]
        self.v2x_target = [0,0,0]
        self.lidar_target = [0,0,0]
        self.target= [0,0,0]
        self.ttc_count = [0,0, 0] # [count if ttc exeed th, all count]
        self.ttc_th  = 2
        self.mode = 0
        self.signal_start = None
        self.detection_time = 0
        

        rospy.Subscriber('/car/mode', Int8, self.mode_cb)
        rospy.Subscriber('/v2x/tlv_pose', Pose, self.tlv_pose_cb)
        rospy.Subscriber('/car/hlv_pose', Pose, self.hlv_pose_cb)
        rospy.Subscriber('/planning/hlv_merged', Int8, self.hlv_merged_cb)
        rospy.Subscriber('/hlv_signal', Int8, self.hlv_signal_cb)
        

    def mode_cb(self, msg):
        if self.mode == 0 and msg.data == 1:
            self.passthorugh_start = time.time()
            self.state = 'start'
            self.mode = 1
        if self.mode == 1 and msg.data == 0:
            self.passthorugh_time = time.time()-self.passthorugh_start
            self.mode = 0
            self.state = 'over'

    def hlv_pose_cb(self, msg):
        self.ego = [msg.position.x, msg.position.y, msg.orientation.x]
    
    def tlv_pose_cb(self, msg):
        self.v2x_target = [msg.position.x, msg.position.y, msg.orientation.x]
        if self.signal_start == None:
            self.detection_time = time.time()-self.signal_start
            self.signal_start = None

    def hlv_merged_cb(self, msg):
        self.hlv_merged = msg.data

    def hlv_signal_cb(self, msg):
        if msg.data == 1 or msg.data == 2:
            self.signal_start = time.time()
        

    def calc_ttc(self):
        ttc = calculate_ttc(ego=self.ego, target=self.target)
        if ttc <= 2:
            self.ttc_count[0] += 1
        if ttc <= 3:
            self.ttc_count[1] += 1
        self.ttc_count[2] += 1

    def logger(self):
        if self.test_mode == 'lidar':
            self.target = self.lidar_target
        elif self.test_mode == 'v2x':
            self.target = self.v2x_target
        
        if self.hlv_merged:
            self.calc_ttc()

    
    def data_set(self):
        self.log_data['bsd detection time'] = round(self.detection_time, 5)
        self.log_data['ttc 2sec cnt'] = self.ttc_count[0]
        self.log_data['ttc 3sec cnt'] = self.ttc_count[1]
        self.log_data['ttc total cnt'] = self.ttc_count[2]
        self.log_data['passthrough time'] = round(self.passthorugh_time, 5)

    def save_to_json(self):
        with open(self.path, 'w') as f:
            json.dump(self.log_data, f, indent=2)

    def run(self):
        rate = rospy.Rate(100)
        while True:
            if self.state  == 'wait':
                pass
            elif self.state == 'start':
                self.logger()
            elif self.state == 'over':
                self.data_set()
                self.save_to_json()
                break
            rate.sleep()


def main():
    rospy.init_node('TestLogger', anonymous=False)
    file_name = sys.argv[1]
    test_mode = sys.argv[2]
    tl = TestLogger(file_name, test_mode)
    tl.run()

if __name__ == "__main__":
    main()