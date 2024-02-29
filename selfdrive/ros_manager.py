import pymap3d as pm
import rospy
from geometry_msgs.msg import Pose,Vector3
from geometry_msgs.msg import Point as GPoint
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from std_msgs.msg import Float32, Int8
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBoxArray

from math import radians

from config.config import Config
from vehicle_state import VehicleState
from localization.point import Point

class RosManager:
    def __init__(self, self_drive, vehicle_type, map):
        config = Config(map)
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
        self.lidar_object = [] 
        self.mode = 0
        self.lidar_dangerous = 0
        self.d = [0,0]
        
    def execute(self):
        print("Start Simulation")
        self.set_protocol()
        
        while not rospy.is_shutdown():
            if self.path != None:
                actuators, local_path, target_velocity, lidar_bsd = self.self_drive.execute(self.mode, self.vehicle_state, self.path, self.lidar_object)
                self.send_data(actuators,local_path, target_velocity, lidar_bsd)
            self.ros_rate.sleep()

    def set_protocol(self):
        rospy.Subscriber(f'/car/{self.vehicle_type}_pose', Pose, self.pose_cb)
        rospy.Subscriber(f'/planning/{self.vehicle_type}_ipath',Marker, self.path_cb)
        rospy.Subscriber('/car/mode', Int8,self.mode_cb)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)
        rospy.Subscriber('/pid_params', Vector3, self.pid_params_cb)
        
        self.actuator_pub = rospy.Publisher(f'/selfdrive/{self.vehicle_type}_actuator', Vector3, queue_size=1)
        self.local_path_pub = rospy.Publisher(f'/selfdrive/{self.vehicle_type}_local_path', Marker, queue_size=1)
        self.target_velocity_pub = rospy.Publisher(f'/selfdrive/{self.vehicle_type}_target_velocity', Float32, queue_size=1)
        self.lidar_bsd_pub = rospy.Publisher(f'/selfdrive/lidar_bsd', Pose, queue_size=1)
        self.pub_lidar_dangerous = rospy.Publisher('/lidar_dangerous', Int8, queue_size=1)


    def conver_to_enu(self, lat, lng):
        x, y, _ = pm.geodetic2enu(lat, lng, 20, self.base_lla['latitude'], self.base_lla['longitude'], self.base_lla['altitude'])
        return x, y

    def mode_cb(self, msg):
        self.mode = msg.data 

    def pose_cb(self, msg):
        x, y = self.conver_to_enu(msg.position.x, msg.position.y)
        self.vehicle_state = VehicleState(x, y, radians(msg.position.z), msg.orientation.x*3.6) # enu x, enu y, radian heading, km/h velocity

    def path_cb(self, msg):
        self.path = [Point(pt.x, pt.y) for pt in msg.points]
    
    def pid_params_cb(self, msg):
        pid_gains = [msg.x, msg.y, msg.z]
        self.self_drive.pid_test(pid_gains)

    def lidar_cluster_cb(self, msg):
        objects = []
        for obj in msg.boxes:
            x, y = obj.pose.position.x, obj.pose.position.y
            v_rel = obj.value #velocity
            track_id = obj.label # 1~: tracking
            w = obj.pose.orientation.z #heading
            objects.append([x, y, w, v_rel, track_id])
        self.lidar_object = objects

    def send_data(self, actuators, local_path, target_velocity, lidar_bsd):
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
        
        self.target_velocity_pub.publish(Float32(target_velocity/3.6))
        
        if len(lidar_bsd) > 0:
            pose = Pose()
            pose.position.x = lidar_bsd[0]
            pose.position.y = lidar_bsd[1]
            pose.position.z = lidar_bsd[2]
            pose.orientation.x = lidar_bsd[3]
            self.lidar_bsd_pub.publish(pose)
            if -30 < lidar_bsd[6] < 30:
                self.lidar_dangerous = 1
            else:
                self.lidar_dangerous = 0
            self.pub_lidar_dangerous.publish(Int8(self.lidar_dangerous))