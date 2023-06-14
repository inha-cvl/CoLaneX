import rospy
from geometry_msgs.msg import Pose
import GPSReader
import OdometryReader


class RosDataPublisher():
    def __init__(self):
        rospy.init_node('ioniq', anonymous=True)
        self.publisher = rospy.Publisher('/car/ioniq_pose', Pose, queue_size=1)
        self.pose = Pose()
        self.gps_reader = GPSReader()
        self.odometry_reader = OdometryReader()
        self.latitude = 0
        self.longitude = 0
        self.yaw = 0
        self.velocity = 0
        self.dummy1 = 0
        self.dummy2 = 0
        self.dummy3 = 0

    def data_read(self):

        self.odometry_reader.read_odom()
        
        self.latitude = self.gps_reader.get_gps()[0]
        self.longitude = self.gps_reader.get_gps()[1]
        self.yaw = self.gps_reader.get_gps()[2]
        self.velocity = self.odometry_reader.get_vel()

    def data_compose(self):

        self.data_read()

        self.pose.position.x = self.latitude
        self.pose.position.y = self.longitude
        self.pose.position.z = self.yaw
        self.pose.orientation.x = self.velocity
        self.pose.orientation.y = self.dummy1
        self.pose.orientation.z = self.dummy2
        self.pose.orientation.w = self.dummy3
    
    def gps_vel_publisher(self):
        self.data_compose()
        self.publisher.publish(self.pose)