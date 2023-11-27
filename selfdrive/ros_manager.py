import pymap3d as pm
import rospy
from geometry_msgs.msg import Pose,Vector3
from geometry_msgs.msg import Point as GPoint
from visualization_msgs.msg import Marker
from math import radians

from config.config import Config
from vehicle_state import VehicleState
from localization.point import Point

class RosManager:
    def __init__(self, self_drive, vehicle_type):
        config = Config()
        self.vehicle_type = vehicle_type
        rospy.init_node(f"{self.vehicle_type}_self_drive", anonymous=True)

        self.sampling_rate = config["common"]["sampling_rate"]
        self.ros_rate = rospy.Rate(self.sampling_rate)
        self.base_lla = config['map']['base_lla']
        if self.vehicle_type == 'hlv':
            self.color = [250, 171, 210]
        else:
            self.color = [168, 232, 255]

        self.self_drive = self_drive
        self.vehicle_state = VehicleState()
        self.path = None
        
    def execute(self):
        print("Start Simulation")
        self.set_protocol()
        
        while not rospy.is_shutdown():
            if self.path != None:
                actuators, local_path = self.self_drive.execute(self.vehicle_state, self.path)
                self.send_data(actuators,local_path)
            self.ros_rate.sleep()

    def set_protocol(self):
        rospy.Subscriber(f'/car/{self.vehicle_type}_pose', Pose, self.pose_cb)
        rospy.Subscriber(f'/planning/{self.vehicle_type}_ipath',Marker, self.path_cb)

        self.actuator_pub = rospy.Publisher(f'/selfdrive/{self.vehicle_type}_actuator', Vector3, queue_size=1)
        self.local_path_pub = rospy.Publisher(f'/selfdrive/{self.vehicle_type}_local_path', Marker, queue_size=1)


    def conver_to_enu(self, lat, lng):
        x, y, _ = pm.geodetic2enu(lat, lng, 20, self.base_lla['latitude'], self.base_lla['longitude'], self.base_lla['altitude'])
        return x, y

    def pose_cb(self, msg):
        x, y = self.conver_to_enu(msg.position.x, msg.position.y)
        self.vehicle_state = VehicleState(x, y, radians(msg.position.z), msg.orientation.x*3.6) # enu x, enu y, radian heading, km/h velocity

    def path_cb(self, msg):
        self.path = [Point(pt.x, pt.y) for pt in msg.points]

    def send_data(self, actuators, local_path):
        vector3 = Vector3()
        vector3.x = actuators.steering
        vector3.y = actuators.accel
        vector3.z = actuators.brake
        self.actuator_pub.publish(vector3)

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = f'{self.vehicle_type}_local_path'
        marker.id = 0
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.7
        marker.color.r = self.color[0]/255
        marker.color.g = self.color[1]/255
        marker.color.b = self.color[2]/255
        marker.color.a = 1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        for pt in local_path:
            marker.points.append(GPoint(x=pt.x, y=pt.y, z=0.5))
        self.local_path_pub.publish(marker)
        


    