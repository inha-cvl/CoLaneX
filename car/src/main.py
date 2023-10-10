# before execute this program
# $ sudo ip link set can0 type can bitrate 500000
# $ sudo ip link set up can0

import rospy
from RosDataPublisher import RosDataPublisher
from Params import ioniq, i30

if __name__ == '__main__':

    # if Hz adjustment is required, code must be added.
    # CAUTION: DO NOT USING rospy sleep.
    # default: ioniq
    ros_data_publisher = RosDataPublisher(**i30)

    try:
        while not rospy.is_shutdown():
            ros_data_publisher.gps_vel_publisher()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Keyboard Interrupt is occured.")

    rospy.spin()
    