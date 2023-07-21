#!/usr/bin/python
import tf
import math
import pymap3d
import sys
import signal
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from visualization_msgs.msg import Marker
from novatel_oem7_msgs.msg import INSPVA
from libs.rviz_utils import *

def signal_handler(sig, frame):
    sys.exit(0)

class Vehicle:
    def __init__(self, x, y, yaw, v, L):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L

    def set(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def next_state(self, dt, wheel_angle, accel, brake):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * dt * math.tan(wheel_angle) / self.L
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        tar_v = self.v

        if accel > 0 and brake == 0:
            tar_v += accel * dt
        elif accel == 0 and brake >= 0:
            tar_v += -brake * dt
        self.v = max(0, tar_v)
        return self.x, self.y, self.yaw, self.v


class TLVSimulator:
    def __init__(self):
        self.base_lla = [35.64588122580907,128.40214778762413, 46.746]
        
        self.ego = Vehicle(-130.220, 416.326, math.radians(142), 0.0, 2.367)
        self.roll = 0.0
        self.pitch = 0.0

        self.target_car = TargetCarViz(-130.220,416.326 )
        self.br = tf.TransformBroadcaster()

        self.pub_target_car = rospy.Publisher('/car/target_car', Marker, queue_size=1)
        # self.pub_novatel = rospy.Publisher('/novatel/oem7/tlv_inspva', INSPVA, queue_size=1)
        # self.pub_velocity = rospy.Publisher('/car/tlv_velocity', Float32, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/tlv_pose', Pose, queue_size=1)
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


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            dt = 0.1
            x, y, yaw, v = self.ego.x, self.ego.y, self.ego.yaw, self.ego.v
            lat, lon, alt = pymap3d.enu2geodetic(x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            v=10.0
            
            # inspva = INSPVA()
            # inspva.latitude = lat
            # inspva.longitude = lon
            # inspva.height = alt
            # inspva.roll = self.roll
            # inspva.pitch = self.pitch
            # self.yaw = -(math.degrees(yaw)+270)
            # inspva.azimuth = self.yaw
            
            # self.pub_novatel.publish(inspva)
            # self.pub_velocity.publish(Float32(v))

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