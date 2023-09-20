import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8, Float32MultiArray

v2x_hlv_pose = rospy.Publisher('/v2x/hlv_pose', Pose, queue_size=1)
v2x_tlv_pose = rospy.Publisher('/v2x/tlv_pose', Pose, queue_size=1)
v2x_hlv_path = rospy.Publisher('/v2x/hlv_path', Marker, queue_size=1)
v2x_tlv_path = rospy.Publisher('/v2x/tlv_path', Marker, queue_size=1)  
hlv_system = rospy.Publisher('/hlv_system', Float32MultiArray, queue_size=1)
tlv_system = rospy.Publisher('/tlv_system', Float32MultiArray, queue_size=1)


def hlv_pose_cb(data):
    v2x_hlv_pose.publish(data)

def tlv_pose_cb(data):
    v2x_tlv_pose.publish(data)

def hlv_path_cb(data):
    v2x_hlv_path.publish(data)

def tlv_path_cb(data):
    v2x_tlv_path.publish(data)

def hlv_state_cb(data):
    _hlv_system = Float32MultiArray()
    _hlv_system.data = [float(data.data), 0, 0, 0, 0]
    hlv_system.publish(_hlv_system)

def tlv_state_cb(data):
    _tlv_system = Float32MultiArray()
    _tlv_system.data = [float(data.data), 0, 0, 0, 0]
    tlv_system.publish(_tlv_system)

def listener():
    rospy.init_node('fake_v2x', anonymous=True)
    rospy.Subscriber('/car/hlv_pose', Pose, hlv_pose_cb)
    rospy.Subscriber('/car/tlv_pose', Pose, tlv_pose_cb)
    rospy.Subscriber('/planning/hlv_path', Marker, hlv_path_cb)
    rospy.Subscriber('/planning/tlv_path', Marker, tlv_path_cb)
    rospy.Subscriber('/hlv_state', Int8, hlv_state_cb)
    rospy.Subscriber('/tlv_state', Int8, tlv_state_cb)
    
    rospy.spin()

if __name__ == '__main__':
    listener()