import os
import tf
import math

import rospy
from visualization_msgs.msg import Marker

dir_path = os.path.dirname(os.path.realpath(__file__))

def CarInfoViz(frame_id, name_space, info, position):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = name_space
    marker.type = Marker.TEXT_VIEW_FACING
    marker.lifetime = rospy.Duration(0)
    marker.scale.z = 2.0
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1.0
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = 3.0
    marker.text = info
    return marker


def CarViz(frame_id, name_space, position, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = name_space
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = 'file://{}/car.dae'.format(dir_path)
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 2.0
    marker.color.r = color[0]/255
    marker.color.g = color[1]/255
    marker.color.b = color[2]/255
    marker.color.a = color[3]
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    quaternion = tf.transformations.quaternion_from_euler(
        0, 0, math.radians(90))
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    return marker
