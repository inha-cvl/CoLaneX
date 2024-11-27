#!/usr/bin/python3
import os
import sys
import signal

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import  Pose
from nav_msgs.msg import Odometry
from novatel_oem7_msgs.msg import INSPVA

over = False
class Car():
    def __init__(self, type):
        
        self.signal = {'left':0, 'right': 0}
        self.pose = Pose() 
        self.pub_pose = rospy.Publisher(f'/car/{type}_pose', Pose, queue_size=1)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        rospy.Subscriber('/novatel/oem7/odom', Odometry, self.novatel_odom_cb)

    def user_mode_cb(self, msg):
        self.user_mode = msg.data
    
    def novatel_cb(self, msg):
        self.pose.position.x = msg.latitude
        self.pose.position.y = msg.longitude
        self.pose.position.z = 89 - msg.azimuth  

    def novatel_odom_cb(self, msg):
        self.pose.orientation.x = msg.twist.twist.linear.x
    
    def publisher(self):
        self.pub_pose.publish(self.pose)


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publisher()
            rate.sleep()
            

def signal_handler(sig, frame):
    over = True 
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Car', anonymous=False)
    type = str(sys.argv[1])
    car = Car(type)
    car.run()
    

if __name__ == "__main__":
    main()
