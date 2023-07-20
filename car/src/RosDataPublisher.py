import rospy
from geometry_msgs.msg import Pose
from GPSReader import GPSReader
from OdometryReader import OdometryReader

class RosDataPublisher():
    def __init__(self, **kwargs):
        
        rospy.init_node(kwargs['publish_init_node'], anonymous=True)
        self.publisher = rospy.Publisher(kwargs['publish_node'], Pose, queue_size=1)
        self.pose = Pose()
        self.gps_reader = GPSReader(ros_subscribe_path = kwargs['ros_subscribe_path'])
        self.odometry_reader = OdometryReader(dbc_file_path = kwargs['dbc_file_path'],
                                              arbitration_id = kwargs['arbitration_id'],
                                              wheel_vel_rr = kwargs['wheel_vel_rr'],
                                              wheel_vel_rl = kwargs['wheel_vel_rl'])
        self.latitude = 0
        self.longitude = 0
        self.yaw = 0
        self.velocity = 0
        self.dummy1 = kwargs['dummy1']
        self.dummy2 = kwargs['dummy2']
        self.dummy3 = kwargs['dummy3']

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