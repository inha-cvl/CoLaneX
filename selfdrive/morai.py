#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from morai_msgs.msg import ObjectStatusList, CtrlCmd, Lamps
from math import radians
from geometry_msgs.msg import Pose, PoseArray, Vector3
from std_msgs.msg import Int8

class Morai():
    def __init__(self):
        self.ctrl_pub = rospy.Publisher(
            '/ctrl_cmd', CtrlCmd, queue_size=1)  # Vehicl Control
        self.lamp_pub = rospy.Publisher('/lamps', Lamps, queue_size=1)
        self.obj_list_pub = rospy.Publisher(
            '/morai/object_list', PoseArray, queue_size=1)
        self.traffic_light_pub = rospy.Publisher(
            '/morai/traffic_light', PoseArray, queue_size=1)
        self.ego_topic_pub = rospy.Publisher(
            '/morai/ego_topic', Pose, queue_size=1)
        rospy.Subscriber('/selfdrive/hlv_actuator', Vector3, self.actuator_cb)
        rospy.Subscriber("/Object_topic", ObjectStatusList,
                         self.object_topic_cb)
        self.ctrl_msg = CtrlCmd()
        self.lamps = Lamps()


    def init_ctrl_cmd(self, ctrl_cmd):
        ctrl_cmd.steering = 0
        ctrl_cmd.accel = 0
        ctrl_cmd.brake = 1.0
        return ctrl_cmd

    def blinker_cb(self, msg):
        self.lamps.turnSignal = msg.data
        self.lamp_pub.publish(self.lamps)

    def rmin(self, x, hi):
        return min(hi, x)

    def actuator_cb(self, data):
        self.ctrl_msg.steering = data.x
        self.ctrl_msg.accel = self.rmin(data.y*5, 100)*0.01
        self.ctrl_msg.brake = self.rmin(data.z*80/65, 100)*0.01

    def object_topic_cb(self, data):
        object_list = PoseArray()
        for obj in data.npc_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        for obj in data.obstacle_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        for obj in data.pedestrian_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        self.obj_list_pub.publish(object_list)

    def run(self):
        self.ctrl_msg = self.init_ctrl_cmd(self.ctrl_msg)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.ctrl_pub.publish(self.ctrl_msg)
            rate.sleep()

def main():
    rospy.init_node('Morai', anonymous=False)
    mp = Morai()
    mp.run()

if __name__ == "__main__":
    main()

