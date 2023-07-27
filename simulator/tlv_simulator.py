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
from rviz_utils import *
from vehicle import Vehicle

def signal_handler(sig, frame):
    sys.exit(0)


class TLVSimulator:
    def __init__(self):
        self.base_lla = [35.64588122580907,128.40214778762413, 46.746]
        
        self.ego = Vehicle(-130.220, 416.326, math.radians(142), 0.0, 2.367)
        self.roll = 0.0
        self.pitch = 0.0

        self.target_car = CarViz('world', 'target_car', [-130.220,416.326,0], [94,204, 243, 1])
        self.br = tf.TransformBroadcaster()


        self.mode = 0
        self._steer = 0
        self._accel = 0
        self._brake = 0


        self.pub_target_car = rospy.Publisher('/car/target_car', Marker, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/tlv_pose', Pose, queue_size=1)
        rospy.Subscriber('/selfdrive/actuator', Vector3, self.actuator_cb)
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
        self._steer = msg.x
        self._accel = msg.y
        self._brake = msg.z
    
    def mode_cb(self, msg):
        self.mode = msg.data


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            dt = 0.1
            if self.mode == 1:
                x, y, yaw, v = self.ego.next_state(
                dt, self._steer,self._accel, self._brake)
            else:
                x, y, yaw, v = self.ego.x, self.ego.y, self.ego.yaw, self.ego.v
            lat, lon, alt = pymap3d.enu2geodetic(x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            v=10.0
            
            pose = Pose()
            pose.position.x = lat
            pose.position.y = lon
            self.yaw = -(math.degrees(yaw)+270)
            pose.position.z = self.yaw
            pose.orientation.x = v
            self.pub_pose.publish(pose)

            self.pub_target_car.publish(self.target_car)

            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('TLVSimulator', anonymous=False)
    st = TLVSimulator()
    st.run()

if __name__ == "__main__":
    main()