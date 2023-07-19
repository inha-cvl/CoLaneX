import rospy
from geometry_msgs.msg import Pose
import GPSReader
import OdometryReader

class RosDataPublisher():
    def __init__(self,
                 dbc_file_path = 'can.dbc',
                 arbitration_id = 640,
                 wheel_vel_rr = 'Gway_Wheel_Velocity_RR',
                 wheel_vel_rl = 'Gway_Wheel_Velocity_RL',
                 ros_subscribe_path= '/novatel/oem7/inspva',
                 publish_init_node = 'ioniq',
                 publish_node = '/car/ioniq_pose',
                 dummy1 = 0,
                 dummy2 = 0,
                 dummy3 = 0):
        
        rospy.init_node(publish_init_node, anonymous=True)
        self.publisher = rospy.Publisher(publish_node, Pose, queue_size=1)
        self.pose = Pose()
        self.gps_reader = GPSReader(ros_subscribe_path = ros_subscribe_path)
        self.odometry_reader = OdometryReader(dbc_file_path = dbc_file_path,
                                              arbitration_id = arbitration_id,
                                              wheel_vel_rr = wheel_vel_rr,
                                              wheel_vel_rl = wheel_vel_rl)
        self.latitude = 0
        self.longitude = 0
        self.yaw = 0
        self.velocity = 0
        self.dummy1 = dummy1
        self.dummy2 = dummy2
        self.dummy3 = dummy3

    def data_read(self) -> None:

        self.odometry_reader.read_odom()
        
        self.latitude = self.gps_reader.get_gps()[0]
        self.longitude = self.gps_reader.get_gps()[1]
        self.yaw = self.gps_reader.get_gps()[2]
        self.velocity = self.odometry_reader.get_vel()

    def data_compose(self) -> None:

        self.data_read()

        self.pose.position.x = self.latitude
        self.pose.position.y = self.longitude
        self.pose.position.z = self.yaw
        self.pose.orientation.x = self.velocity
        self.pose.orientation.y = self.dummy1
        self.pose.orientation.z = self.dummy2
        self.pose.orientation.w = self.dummy3
    
    def gps_vel_publisher(self) -> None:
        self.data_compose()
        self.publisher.publish(self.pose)