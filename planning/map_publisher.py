import sys
import signal
import rospy
from visualization_msgs.msg import MarkerArray
from libs.map import LaneletMap
from libs.rviz_utils import LaneletMapViz

def signal_handler(sig, frame):
    sys.exit(0)

class MapPublisher:
    def __init__(self):
        #self.base_lla = [35.64588122580907,128.40214778762413, 46.746] #KIAPI
        self.base_lla = [37.383378,126.656798,7] # Sondo-Site
        lmap = LaneletMap("songdo-site.json")
        
        self.lanelet_map_viz = LaneletMapViz(lmap.lanelets, lmap.for_viz)
        self.pub_lanelet_map = rospy.Publisher('/planning/lanelet_map', MarkerArray, queue_size = 1, latch=True)

    def run(self):
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown():
            self.pub_lanelet_map.publish(self.lanelet_map_viz)
            rate.sleep()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('MAP', anonymous=False)
    m = MapPublisher()
    m.run()

if __name__ == "__main__":
    main()