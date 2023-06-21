import rospy
import RosDataPublisher
from Params import ioniq, i30

if __name__ == '__main__':

    
    # if Hz adjustment is required, code must be added.
    # CAUTION: DO NOT USING rospy sleep.
    # default: ioniq
    ros_data_publisher = RosDataPublisher()

    try:
        while not rospy.is_shutdown():
            ros_data_publisher.gps_vel_publisher()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
