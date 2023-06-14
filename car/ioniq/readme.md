GPSReader        : Read GPS  
OdometryReader   : Read CAN  
RosDataPublisher : Compose data for ROS topic and publish data  
ioniq_main       : main file  

  
  ### GPSReader
  - Read RTK GPS with Novatel
  - When RTK is not working, it return 0
  - yaw : 89 - azimuth
    

  ### OdometryReader
  - Read CAN with can.dbc
  - if you do not set the timeout for the recv(), no output.
  - The velocity is the average of the velocity of the right and left rear wheels. (RR + RL) / 2
    

  ### RosDataPublisher
  - Using ROS Pose class
  - Pose.position.x = latitude
  - Pose.position.y = longitude
  - Pose.position.z = yaw (89 - azimuth)
  - Pose.orientation.x = velocity ((RR+RL)/2)
  - Pose.orientation.y ~ w = dummy1 ~ 3

  ### ioniq_main
  - Only need to run this.
  - If use rospy.rate.sleep(), CAN IS NOT WORKING.
