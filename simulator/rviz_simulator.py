#!/usr/bin/python
import tf
import math
import pymap3d
import sys
import signal
import rospy
from geometry_msgs.msg import  Pose
from visualization_msgs.msg import Marker
from libs.rviz_utils import *

def signal_handler(sig, frame):
    sys.exit(0)

class RVizSimulator:
    def __init__(self, vehicle_type, map):
        if map == 'songdo-site':
            self.base_lla = [37.383378,126.656798,7] # Sondo-Site
        elif map == 'KIAPI':
            self.base_lla = [35.64588122580907,128.40214778762413, 46.746]
        else:
            self.base_lla = [37.2292221592864,126.76912499027308,29.18400001525879]

        another_type = 'tlv' if vehicle_type == 'hlv' else 'hlv'
        car_color = [241, 76, 152, 1] if vehicle_type == 'hlv' else [94,204, 243, 1]
        a_car_color = [94,204, 243, 1] if vehicle_type == 'hlv' else [241, 76, 152, 1]
        l_car_color = [94, 243, 126, 1]

        self.car_marker = CarViz('car', 'car', [0, 0, 0], car_color)
        self.car_info = CarInfoViz('car', 'car_info', '',[0,0,0])
        self.a_car_marker = CarViz('world', 'a_car', [0, 0, 0], a_car_color)
        self.a_car_info = CarInfoViz('world', 'a_car_info', '',[0,0,0])
        self.l_car_marker = CarViz('world', 'l_car', [0, 0, 0], l_car_color)
        self.l_car_info = CarInfoViz('world', 'l_car_info', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        self.x, self.y, self.h, self.v = 0, 0, 0, 0
        self.a_x, self.a_y, self.a_h, self.a_v = 0, 0, 0, 0
        self.l_x, self.l_y, self.l_h, self.l_v = 0, 0, 0, 0

        rospy.init_node(f'{vehicle_type}_simulator', anonymous=False)
        self.pub_car_marker = rospy.Publisher(f'/simulator/{vehicle_type}_marker', Marker, queue_size=1)
        self.pub_car_info = rospy.Publisher(f'/simulator/{vehicle_type}_info', Marker, queue_size=1)
        self.pub_car_path = rospy.Publisher(f'/simulator/{vehicle_type}_path', Marker, queue_size=1)
        self.pub_a_car_marker = rospy.Publisher(f'/simulator/{another_type}_marker', Marker, queue_size=1)
        self.pub_a_car_info = rospy.Publisher(f'/simulator/{another_type}_info', Marker, queue_size=1)
        self.pub_a_car_path = rospy.Publisher(f'/simulator/{another_type}_path', Marker, queue_size=1)
        self.pub_l_car_marker = rospy.Publisher(f'/simulator/lidar_marker', Marker, queue_size=1)
        self.pub_l_car_info = rospy.Publisher(f'/simulator/lidar_info', Marker, queue_size=1)
        rospy.Subscriber(f'/car/{vehicle_type}_pose', Pose, self.pose_cb)
        rospy.Subscriber(f'/v2x/{another_type}_pose', Pose, self.another_pose_cb)
        rospy.Subscriber(f'/planning/{vehicle_type}_path', Marker, self.path_cb)
        rospy.Subscriber(f'/v2x/{another_type}_path', Marker, self.another_path_cb)
        rospy.Subscriber(f'/selfdrive/lidar_bsd', Pose, self.lidar_bsd_cb)


    def pose_cb(self, msg):
        self.x, self.y, _ = pymap3d.geodetic2enu(msg.position.x, msg.position.y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.h = msg.position.z
        self.v = msg.orientation.x 
    
    def another_pose_cb(self, msg):
        self.a_x, self.a_y, _ = pymap3d.geodetic2enu(msg.position.x, msg.position.y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.a_h = msg.position.z
        self.a_v = msg.orientation.x 
    
    def lidar_bsd_cb(self, msg):
        self.l_x, self.l_y, _ = pymap3d.geodetic2enu(msg.position.x, msg.position.y,0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.l_h = msg.position.z
        self.l_v = msg.orientation.x 
    
    def path_cb(self, msg):
        self.pub_car_path.publish(msg)
    
    def another_path_cb(self, msg):
        self.pub_a_car_path.publish(msg)
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            info = f"{(self.v*3.6):.2f}km/h {self.h:.2f}deg"
            self.car_info.text = info
            a_info = f"{(self.a_v*3.6):.2f}km/h {self.a_h:.2f}deg"
            self.a_car_info.text = a_info
            l_info = f"{(self.l_v*3.6):.2f}km/h {self.l_h:.2f}deg"
            self.l_car_info.text = l_info

            quaternion = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(self.h))  # RPY
            self.br.sendTransform(
                (self.x, self.y, 0),
                (quaternion[0], quaternion[1],
                    quaternion[2], quaternion[3]),
                rospy.Time.now(),
                'car',
                'world'
            )
            self.a_car_marker.pose.position.x = self.a_x
            self.a_car_marker.pose.position.y = self.a_y 
            self.a_car_info.pose.position.x = self.a_x
            self.a_car_info.pose.position.y = self.a_y 

            self.l_car_marker.pose.position.x = self.l_x
            self.l_car_marker.pose.position.y = self.l_y 
            self.l_car_info.pose.position.x = self.l_x
            self.l_car_info.pose.position.y = self.l_y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(90+(self.l_h%360)))
            self.l_car_marker.pose.orientation.x = quaternion[0]
            self.l_car_marker.pose.orientation.y = quaternion[1]
            self.l_car_marker.pose.orientation.z = quaternion[2]
            self.l_car_marker.pose.orientation.w = quaternion[3]

            self.pub_car_marker.publish(self.car_marker)
            self.pub_car_info.publish(self.car_info)
            self.pub_a_car_marker.publish(self.a_car_marker)
            self.pub_a_car_info.publish(self.a_car_info)
            self.pub_l_car_marker.publish(self.l_car_marker)
            self.pub_l_car_info.publish(self.l_car_info)

            rate.sleep()

def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ['hlv', 'tlv']:
        print("Usage: python your_script_name.py <vehicle_type> (hlv or tlv)")
        return
    vehicle_type = sys.argv[1]
    map = sys.argv[2]
    signal.signal(signal.SIGINT, signal_handler)
    st = RVizSimulator(vehicle_type, map)
    st.run()

if __name__ == "__main__":
    main()