#!/usr/bin/python
import tf
import math
import pymap3d
import sys
import signal
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8
from libs.rviz_utils import *
from libs.vehicle import Vehicle

def signal_handler(sig, frame):
    sys.exit(0)

class HLVSimulator:
    def __init__(self, map):
        wheelbase = 2.97
        if map == 'songdo-site':
            self.base_lla = [37.383378,126.656798,7] # Sondo-Site
            self.ego = Vehicle(-3800.520, 3840.930, -3.133, 0.0, wheelbase) #Songdo
        elif map=='songdo':
            self.base_lla = [37.3888319,126.6428739, 7.369]
            #self.ego = Vehicle(148.707, 310.741,2.478, 0.0, 2.65)
            self.ego = Vehicle(147.547, 311.788, 2.478, 0.0, wheelbase)  
        elif map == 'KIAPI':
            self.base_lla = [35.64588122580907,128.40214778762413, 46.746]
            self.ego = Vehicle(-127.595, 418.819, 2.380, 0.0, wheelbase)
        elif map == 'Pangyo':
            self.base_lla = [37.39991792889962, 127.11264200835348,7]
            self.ego = Vehicle(-10.687, 0.029, -3.079,0,wheelbase)
        else:
            self.base_lla = [37.2292221592864,126.76912499027308,29.18400001525879]
            self.ego = Vehicle(533.086,1779.300,-1.577,0, wheelbase)

        self.roll = 0.0
        self.pitch = 0.0

        self.ego_car = CarViz('ego_car', 'ego_car_info', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        self.mode = 0
        self._steer = 0
        self._accel = 0
        self._brake = 0

        self.pub_ego_car = rospy.Publisher('/car/ego_car', Marker, queue_size=1)
        self.pub_ego_car_info = rospy.Publisher('/car/ego_car_info', Marker, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/hlv_pose', Pose, queue_size=1)
        self.pub_mode = rospy.Publisher('/car/mode', Int8, queue_size=1)
        rospy.Subscriber('/selfdrive/hlv_actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/mode', Int8, self.mode_cb)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        self.roll, self.pitch, yaw = tf.transformations.euler_from_quaternion(
            quaternion)
        self.ego.set(x, y, yaw)

    def actuator_cb(self, msg):
        self._steer = math.radians(msg.x)
        self._accel = msg.y
        self._brake = msg.z
    
    def mode_cb(self, msg):
        self.mode = msg.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            dt = 0.05
            if self.mode == 1:
                x, y, yaw, v = self.ego.next_state(
                dt, self._steer,self._accel, self._brake)
            else:
                x, y, yaw, v = self.ego.x, self.ego.y, self.ego.yaw, self.ego.v
            #v = 12.0 # 40km/h 
            lat, lon, alt = pymap3d.enu2geodetic(x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])

            pose = Pose()
            pose.position.x = lat
            pose.position.y = lon
            self.yaw = math.degrees(yaw)
            pose.position.z = self.yaw
            pose.orientation.x = v
            self.pub_pose.publish(pose)

            info = f"{(v*3.6):.2f}km/h {self.yaw:.2f}deg"
            self.ego_car_info.text = info

            quaternion = tf.transformations.quaternion_from_euler(math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw))  # RPY
            self.br.sendTransform(
                (x, y, 0),
                (quaternion[0], quaternion[1],
                    quaternion[2], quaternion[3]),
                rospy.Time.now(),
                'ego_car',
                'world'
            )
            self.pub_ego_car.publish(self.ego_car)
            self.pub_ego_car_info.publish(self.ego_car_info)
            self.pub_mode.publish(Int8(self.mode))
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('HLVSimulator', anonymous=False)
    map = sys.argv[1]
    st = HLVSimulator(map)
    st.run()

if __name__ == "__main__":
    main()