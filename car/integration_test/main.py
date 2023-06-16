import rospy
import RosDataPublisher

if __name__ == '__main__':

    ioniq = {'id' : 1,
             'dbc_file_path' : 'can.dbc',
             'arbitration_id' : 640,
             'wheel_vel_rr' : 'Gway_Wheel_Velocity_RR',
             'wheel_vel_rl' : 'Gway_Wheel_Velocity_RL',
             'ros_subscribe_path' : '/novatel/oem7/inspva',
             'publish_init_node' : 'ioniq',
             'publish_node' : '/car/ioniq_pose',
             'dummy1' : 0,
             'dummy2' : 0,
             'dummy3' : 0
             }
    
    i30 = {{'id' : 2,
             'dbc_file_path' : 'huyndai_2015_ccan.dbc',
             'arbitration_id' : 902,
             'wheel_vel_rr' : 'WHL_SPD_RR',
             'wheel_vel_rl' : 'WHL_SPD_RL',
             'ros_subscribe_path' : '/novatel/oem7/inspva',
             'publish_init_node' : 'i30',
             'publish_node' : '/car/i30_pose',
             'dummy1' : 0,
             'dummy2' : 0,
             'dummy3' : 0
             }}
    
    ros_data_publisher = RosDataPublisher()
    # rate = rospy.Rate(10) # 10Hz

    try:
        while not rospy.is_shutdown():
            ros_data_publisher.gps_vel_publisher()
            # rate.sleep()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
