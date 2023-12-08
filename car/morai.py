#!/usr/bin/python
import math
import tf
import rospy
from geometry_msgs.msg import Pose
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from sensor_msgs.msg import Imu
from morai_msgs.msg import ObjectStatusList, CtrlCmd, Lamps
from geometry_msgs.msg import Pose, PoseArray, Vector3
from std_msgs.msg import Int8
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox


class Morai:
    def __init__(self):
 
        self.pose = Pose()
        self.ctrl_msg = CtrlCmd()
        self.lamps = Lamps()
        self.mode = 0
        self.egoxy = [0,0]
                
        self.pub_pose = rospy.Publisher('/car/hlv_pose', Pose, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)  # Vehicl Control
        self.lamp_pub = rospy.Publisher('/lamps', Lamps, queue_size=1)
        self.obj_list_pub = rospy.Publisher('/mobinha/perception/lidar/track_box', BoundingBoxArray, queue_size=1)
        self.mode_pub = rospy.Publisher('/car/mode', Int8, queue_size=1)
        rospy.Subscriber("/gps", GPSMessage, self.gps_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)
        rospy.Subscriber("/Object_topic", ObjectStatusList,self.object_topic_cb)
        rospy.Subscriber('/selfdrive/hlv_actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/hlv_signal', Int8, self.hlv_signal_cb)
        rospy.Subscriber('/mode', Int8, self.mode_cb)
        
    def gps_cb(self, msg):
        self.pose.position.x = msg.latitude
        self.pose.position.y = msg.longitude
    
    def mode_cb(self, msg):
        self.mode = msg.data

    def imu_cb(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y,msg.orientation.z, msg.orientation.w)
        _, _,yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.position.z = math.degrees(yaw)

    def ego_topic_cb(self, msg):
        self.pose.orientation.x = msg.velocity.x
        self.pose.orientation.y = msg.wheel_angle
        self.pose.orientation.z = msg.accel
        self.pose.orientation.w = msg.brake

        self.egoxy[0] = msg.position.x
        self.egoxy[1] = msg.position.y

    def hlv_signal_cb(self, msg):
        self.lamps.turnSignal = msg.data

    def actuator_cb(self, data):
        self.ctrl_msg.steering = math.radians(data.x)
        self.ctrl_msg.accel = data.y
        self.ctrl_msg.brake = data.z

    def object_topic_cb(self, data):
        object_list = BoundingBoxArray()
        for obj in data.npc_list:
            bbox = BoundingBox()
            pose = Pose()
            pose.position.y = -(self.egoxy[0] - obj.position.x)
            pose.position.x = self.egoxy[1] - obj.position.y
            pose.orientation.z = obj.heading
            bbox.pose = pose
            bbox.value = obj.velocity.x/3.6
            bbox.label = 1
            object_list.boxes.append(bbox)
        for obj in data.obstacle_list:
            bbox = BoundingBox()
            pose = Pose()
            pose.position.y = -(self.egoxy[0] - obj.position.x)
            pose.position.x = self.egoxy[1] - obj.position.y
            pose.orientation.z = obj.heading
            bbox.pose = pose
            bbox.value = obj.velocity.x/3.6
            bbox.label = 1
            object_list.boxes.append(bbox)
        self.obj_list_pub.publish(object_list)

    def publisher(self):
        self.pub_pose.publish(self.pose)
        self.ctrl_pub.publish(self.ctrl_msg)
        self.lamp_pub.publish(self.lamps)
        self.mode_pub.publish(Int8(self.mode))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publisher()
            rate.sleep()
    
def main():
    rospy.init_node('Morai', anonymous=False)
    m = Morai()
    m.run()
    
if __name__ == "__main__":
    main()