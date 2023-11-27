#!/usr/bin/python
import math
import tf

import rospy
from geometry_msgs.msg import Pose
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from sensor_msgs.msg import Imu

class MoraiCar:
    def __init__(self):
        self.gps_ok = False
        self.imu_ok = False

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pub_pose = rospy.Publisher('/car/hlv_pose', Pose, queue_size=1)
        #From MORAI
        rospy.Subscriber("/gps", GPSMessage, self.gps_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)

    def gps_cb(self, msg):
        self.gps_ok = True
        self.ll_position = (msg.latitude, msg.longitude, msg.altitude)

    def imu_cb(self, msg):
        self.imu_ok = True
        quaternion = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(
            quaternion)

    def ego_topic_cb(self, msg):
        self.imu_ok = True
        self.heading = msg.heading
        self.velocity = msg.velocity.x

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.gps_ok and self.imu_ok:
                pose = Pose()
                pose.position.x = self.ll_position[0]
                pose.position.y = self.ll_position[1]
                pose.position.z = math.degrees(self.yaw)
                pose.orientation.x = self.velocity
                self.pub_pose.publish(pose)
            rate.sleep()
    
def main():
    rospy.init_node('MoraiCar', anonymous=False)
    mc = MoraiCar()
    mc.run()
    
if __name__ == "__main__":
    main()