import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose


class UITest:
    def __init__(self):
        rospy.init_node('UITest', anonymous=True)
        self.pub_system = rospy.Publisher('/hlv_system', Float32MultiArray, queue_size=1)
        self.test_system = Float32MultiArray() 
        self.pub_position = rospy.Publisher('/hlv_position', Pose, queue_size=1)
        self.test_position = Pose()


    def run(self):  
        rate = rospy.Rate(10) #Publish Message 10Hz
        while not rospy.is_shutdown():
            self.test_system.data = [0, 0, 0, 0, 0] #state, GPS, V2X, Latency, Speed
            self.test_position.position.x = 35.64588122580907 #Latitude
            self.test_position.position.y = 128.40214778762413 #Longitude
            self.test_position.position.z = 46.746 #Altitude
            
            self.pub_system.publish(self.test_system)
            self.pub_position.publish(self.test_position)
            rate.sleep()


def main():
    ut = UITest()
    ut.run()

if __name__ == "__main__":
    main()
