import rospy
from novatel_oem7_msgs.msg import INSPVA
from sensor_msgs.msg import NavSatFix, IMU

class GPSReader:
    def __init__(self,
                 ros_subscribe_path = '/novatel/oem7/inspva',
                 ):
        
        if (ros_subscribe_path == '/novatel/oem7/inspva'):
            rospy.Subscriber(ros_subscribe_path, INSPVA, self.novatel_callback)
        elif (ros_subscribe_path == '/vectornav/gps'):
            rospy.Subscriber(ros_subscribe_path, NavSatFix, self.vectornav_gps_callback)
            rospy.Subscriber('/vectornav/IMU', IMU, self.vectornav_imu_callback)
        else:
            print("===== ROS subscribe path is wrong =====")
            print("===== Set Params to IONIQ =====")
            rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_callback)
    
        self.latitude = 0
        self.longitude = 0
        self.yaw = 0

    def novatel_callback(self, msg) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.yaw = 89 - msg.azimuth
    
    def vectornav_gps_callback(self, msg) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
    
    def vectornav_imu_callback(self, msg) -> None:
        self.yaw = msg.yaw

    # return: [latitude, longitude, yaw]
    def get_gps(self) -> list:
        carGPS = [self.latitude, self.longitude, self.yaw]

        return carGPS