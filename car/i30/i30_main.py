import rospy
import RosDataPublisher

if __name__ == '__main__':
    
    ros_data_publisher = RosDataPublisher()
    # rate = rospy.Rate(10) # 10Hz

    try:
        while not rospy.is_shutdown():
            ros_data_publisher.gps_vel_publisher()
            # rate.sleep()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
