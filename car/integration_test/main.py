import rospy
import RosDataPublisher, Params

if __name__ == '__main__':

    
    # if Hz adjustment is required, code must be added.
    # CAUTION: DO NOT USING rospy sleep.
    # ioniq -> params.ioniq
    # i30 -> parmas.ioniq
    params = Params()
    ros_data_publisher = RosDataPublisher(**params.ioniq)

    try:
        while not rospy.is_shutdown():
            ros_data_publisher.gps_vel_publisher()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
