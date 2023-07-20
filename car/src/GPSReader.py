import rospy
from novatel_oem7_msgs.msg import INSPVA
from sensor_msgs.msg import NavSatFix, Imu
from math import atan2, pi

class GPSReader:
    def __init__(self,
                 ros_subscribe_path = '/novatel/oem7/inspva',
                 ):
        
        if (ros_subscribe_path == '/novatel/oem7/inspva'):
            rospy.Subscriber(ros_subscribe_path, INSPVA, self.novatel_callback)
        elif (ros_subscribe_path == '/vectornav/gps'):
            rospy.Subscriber(ros_subscribe_path, NavSatFix, self.vectornav_gps_callback)
            rospy.Subscriber('/vectornav/IMU', Imu, self.vectornav_imu_callback)
        else:
            print("===== ROS subscribe path is wrong =====")
    
        self.latitude = 0
        self.longitude = 0
        self.yaw = 0

    def novatel_callback(self, msg) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        # 90        : for ENU(East North) coordinate
        # 89        : for IONIQ antenna bias
        # -azimuth  : for ccw
        self.yaw = 89 - msg.azimuth
    
    def vectornav_gps_callback(self, msg) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def vectornav_imu_callback(self, msg) -> None:
        self.yaw = self.quaternion_to_euler_yaw_to_deg(msg.orientation.x,
                                                       msg.orientation.y,
                                                       msg.orientation.z,
                                                       msg.orientation.w)

    # return: [latitude, longitude, yaw]
    def get_gps(self) -> list:
        carGPS = [self.latitude, self.longitude, self.yaw]

        return carGPS
    
    def quaternion_to_euler_yaw_to_deg(self, x, y, z, w) -> float:
        a = 2.0 * (w * z + x * y)
        b = 1.0 - 2.0 * (y * y + z * z)
        
        # for ENU coordinate
        # convert rad to deg
        return 90 - (atan2(a, b) / pi * 180)