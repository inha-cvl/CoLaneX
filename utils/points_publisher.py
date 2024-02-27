import rospy
from visualization_msgs.msg import Marker


def Sphere(ns, id_, data, scale, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = 0
    marker.color.r = color[0]/255
    marker.color.g = color[1]/255
    marker.color.b = color[2]/255
    marker.color.a = 0.9
    marker.pose.position.x = data[0]
    marker.pose.position.y = data[1]
    marker.pose.position.z = 0
    return marker


#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def publish_sphere_marker():
    rospy.init_node('sphere_marker_publisher', anonymous=True)
    
    # Marker를 발행할 퍼블리셔 생성
    pub_marker_array = rospy.Publisher('point_array', MarkerArray, queue_size=10)
    
    # 1초마다 반복
    rate = rospy.Rate(1)
    
    start_2 = [532.866, 1791.557]
    end_2 = [544.660, 1381.380]
    start_1 = [536.182, 1790.470]
    end_1 = [548.158, 1381.574]

    #TLV pos
    #hlv behind at lane 2
    lane_1_35 = [536.7168486141295, 1754.697308443755] #35km/h 
    lane_1_50 = [536.8457533338291, 1738.6976545092748] #50km/h 
    lane_1_65 = [537.0125364209297, 1724.1981381865805] #65km/h 
    #hlv front at lane 2
    lane_2_35 = [534.0112302335458, 1754.6435574855586] #35km/h
    lane_2_50 = [534.5905010296598, 1739.7620290075165] #50km/h
    lane_2_65 = [535.1677337063974, 1724.8445565868524] #65km/h

    marker_array = MarkerArray()
    marker_array.markers.append(Sphere('start', 1, start_1, 4.0, (0,0,255)))
    marker_array.markers.append(Sphere('end', 1, end_1, 4.0, (0,0,255)))
    marker_array.markers.append(Sphere('start', 2, start_2, 4.0, (255, 0, 0)))
    marker_array.markers.append(Sphere('end', 2, end_2, 4.0, (255,0,0)))

    marker_array.markers.append(Sphere('35kmh', 1, lane_1_35, 4.0, (0, 255, 195)))
    marker_array.markers.append(Sphere('50kmh', 1, lane_1_50, 4.0, (0, 255, 195)))
    marker_array.markers.append(Sphere('65kmh', 1, lane_1_65, 4.0, (0, 255, 195)))

    marker_array.markers.append(Sphere('35kmh', 2, lane_2_35, 4.0, (255,145,0)))
    marker_array.markers.append(Sphere('50kmh', 2, lane_2_50, 4.0, (255,145,0)))
    marker_array.markers.append(Sphere('65kmh', 2, lane_2_65, 4.0, (255,145,0)))




    
    while not rospy.is_shutdown(): 
        pub_marker_array.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sphere_marker()
    except rospy.ROSInterruptException:
        pass
