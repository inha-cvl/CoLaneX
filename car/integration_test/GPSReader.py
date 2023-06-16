import rospy
from novatel_oem7_msgs.msg import INSPVA

class GPSReader:
    def __init__(self):
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_callback)
        self.latitude = 0
        self.longitude = 0
        self.yaw = 0

    def novatel_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.yaw = 89 - msg.azimuth

    # return: [latitude, longitude, yaw]
    def get_gps(self) -> list:
        carGPS = [self.latitude, self.longitude, self.yaw]

        return carGPS