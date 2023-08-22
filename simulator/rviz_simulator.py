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
    def __init__(self, vehicle_type):
        #self.base_lla = [35.64588122580907,128.40214778762413, 46.746] #KIAPI
        self.base_lla = [37.383378,126.656798,7] # Sondo-Site

        another_type = 'tlv' if vehicle_type == 'hlv' else 'hlv'
        car_color = [241, 76, 152, 1] if vehicle_type == 'hlv' else [94,204, 243, 1]
        a_car_color = [94,204, 243, 1] if vehicle_type == 'hlv' else [241, 76, 152, 1]

        self.car_marker = CarViz('car', 'car', [0, 0, 0], car_color)
        self.car_info = CarInfoViz('car', 'car_info', '',[0,0,0])
        self.a_car_marker = CarViz('world', 'a_car', [0, 0, 0], a_car_color)
        self.a_car_info = CarInfoViz('world', 'a_car_info', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        self.x, self.y, self.h, self.v = 0, 0, 0, 0
        self.a_x, self.a_y, self.a_h, self.a_v = 0, 0, 0, 0
        

        rospy.init_node(f'{vehicle_type}_simulator', anonymous=False)
        self.pub_car_marker = rospy.Publisher(f'/simulator/{vehicle_type}_marker', Marker, queue_size=1)
        self.pub_car_info = rospy.Publisher(f'/simulator/{vehicle_type}_info', Marker, queue_size=1)
        self.pub_a_car_marker = rospy.Publisher(f'/simulator/{another_type}_marker', Marker, queue_size=1)
        self.pub_a_car_info = rospy.Publisher(f'/simulator/{another_type}_info', Marker, queue_size=1)
        rospy.Subscriber(f'/car/{vehicle_type}_pose', Pose, self.pose_cb)
        rospy.Subscriber(f'/v2x/{another_type}_pose', Pose, self.another_pose_cb)


    def pose_cb(self, msg):
        self.x, self.y, _ = pymap3d.geodetic2enu(msg.position.x, msg.position.y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.h = msg.position.z
        self.v = msg.orientation.x 
    
    def another_pose_cb(self, msg):
        self.a_x, self.a_y, _ = pymap3d.geodetic2enu(msg.position.x, msg.position.y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.a_h = msg.position.z
        self.a_v = msg.orientation.x 
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            info = f"{(self.v*3.6):.2f}km/h {self.h:.2f}deg"
            self.car_info.text = info
            a_info = f"{(self.a_v*3.6):.2f}km/h {self.a_h:.2f}deg"
            self.a_car_info.text = a_info

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

            self.pub_car_marker.publish(self.car_marker)
            self.pub_car_info.publish(self.car_info)
            self.pub_a_car_marker.publish(self.a_car_marker)
            self.pub_a_car_info.publish(self.a_car_info)

            rate.sleep()

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ['hlv', 'tlv']:
        print("Usage: python your_script_name.py <vehicle_type> (hlv or tlv)")
        return
    vehicle_type = sys.argv[1]
    signal.signal(signal.SIGINT, signal_handler)
    st = RVizSimulator(vehicle_type)
    st.run()

if __name__ == "__main__":
    main()