#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import pymap3d as pm
import math
import numpy as np
from jsk_recognition_msgs.msg import BoundingBoxArray
from geopy.distance import great_circle

class DistanceCalculatorWithGeopy:
    def __init__(self):
        rospy.init_node('distance_calculator_with_geopy', anonymous=True)
        self.pose_sub1 = rospy.Subscriber("/car/hlv_pose", Pose, self.pose_callback1)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.pose_callback2)
        self.pos1 = self.pos2 = self.ppos1 = None
        self.base_lla=[
		37.2292221592864,
		126.76912499027308,
		29.18400001525879]

    def pose_callback1(self, msg):
        # 위도와 경도를 position.x와 position.y로 가정
        self.pos1 = (msg.position.x, msg.position.y)
        self.heading = msg.position.z
        x, y, _ = pm.geodetic2enu(self.pos1[0], self.pos1[1], 10, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.ppos1 = (x,y)
        print("ENU", self.ppos1)
        self.calculate_distance()

    def object2enu(self, odom, obj_local_x, obj_local_y):
        rad = np.radians(odom[2])

        nx = math.cos(rad) * obj_local_x - math.sin(rad) * obj_local_y
        ny = math.sin(rad) * obj_local_x + math.cos(rad) * obj_local_y

        obj_x = odom[0] + nx
        obj_y = odom[1] + ny

        return obj_x, obj_y

    def pose_callback2(self, msg):
        # 위도와 경도를 position.x와 position.y로 가정
        if self.ppos1 is not None:
            for obj in msg.boxes:
                
                nx, ny = self.object2enu((self.ppos1[0], self.ppos1[1], self.heading), obj.pose.position.x, obj.pose.position.y)
                #print('ENU = ', nx, ny)
                lat, lng, _ = pm.enu2geodetic(nx,ny, 10, self.base_lla[0], self.base_lla[1], self.base_lla[2])
                self.pos2 = (lat, lng)
                break
            self.calculate_distance()

    

    def calculate_distance(self):
        if self.pos1 is not None and self.pos2 is not None:
            distance = great_circle(self.pos1, self.pos2).meters 
            rospy.loginfo(f"Distance: {distance} m")

if __name__ == '__main__':
    try:
        DistanceCalculatorWithGeopy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
