import rospy
import random
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose


class UITest:
    def __init__(self):
        rospy.init_node('UITest', anonymous=True)
        self.pub_system = rospy.Publisher('/hlv_system', Float32MultiArray, queue_size=1)
        self.test_system = Float32MultiArray() 
        
        self.pub_position = rospy.Publisher('/hlv_position', Pose, queue_size=1)
        self.test_position = Pose()
        self.pub_geojson = rospy.Publisher('/hlv_geojson', String, queue_size=1)
        self.geojson = ""
        self.cnt = 0


    def run(self):  
        rate = rospy.Rate(10) #Publish Message 10Hz
        state = 0
        gps = 0
        v2x = 0
        latency = 0
        speed = 0

        while not rospy.is_shutdown():
            if self.cnt == 50:
                state = random.randint(0, 5)
                gps = random.randint(0,1)
                v2x = random.randint(0,1)
                latency = random.randint(0,3000)
                speed = random.randint(0, 300)
                self.cnt = 0

            self.test_system.data = [state, gps, v2x, latency, speed] #state, GPS, V2X, Latency, Speed
            self.test_position.position.x = 35.64588122580907 #Latitude
            self.test_position.position.y = 128.40214778762413 #Longitude
            self.test_position.position.z = 0 #Heading (deg)
            self.test_position.orientation.x = 3 #Speed (m/s)
            
            self.pub_system.publish(self.test_system)
            #self.pub_position.publish(self.test_position)
            self.cnt += 1
            rate.sleep()


def main():
    ut = UITest()
    ut.run()

if __name__ == "__main__":
    main()
