import rospy
import random
import pymap3d as pm
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8, Float32MultiArray

v2x_hlv_pose = rospy.Publisher('/v2x/hlv_pose', Pose, queue_size=1)
v2x_tlv_pose = rospy.Publisher('/v2x/tlv_pose', Pose, queue_size=1)
v2x_hlv_path = rospy.Publisher('/v2x/hlv_path', Marker, queue_size=1)
v2x_tlv_path = rospy.Publisher('/v2x/tlv_path', Marker, queue_size=1)  
hlv_system = rospy.Publisher('/hlv_system', Float32MultiArray, queue_size=1)
tlv_system = rospy.Publisher('/tlv_system', Float32MultiArray, queue_size=1)
hlv_signal_pub = rospy.Publisher('/hlv_signal', Int8, queue_size=1)

hlv_signal = 0
tlv_signal = 0
#(-319.981,  -301.069)
click_point = [(-241.434, 610.810),  (-605.899, 910.478), (-697.889, 359.962), (-394.124, -191.119),
               (-306.069, -303.600), (-128.979, -134.587), (-161.371, -182.209), (8.831, 21.282), (148.024, 187.529),  (167.265, 292.585), (88.748, 357.170)]
def hlv_signal_pub_songdo(data):
    signal = 0
    x, y, _ = pm.geodetic2enu(data.position.x, data.position.y, 20,  37.3888319, 126.6428739, 7.369)
    for i, pt in enumerate(click_point):
        distance = math.sqrt((x-pt[0])**2+(y-pt[1])**2)
        if distance < 1:
            if i == len(click_point)-2:
                signal = 2
            elif i == len(click_point)-1:
                signal = 3
            else:
                signal = 1 
            break
    return signal

def hlv_pose_cb(data):
    signal = hlv_signal_pub_songdo(data)
    
    data.orientation.y = signal
    v2x_hlv_pose.publish(data)
    hlv_signal_pub.publish(Int8(signal))

def tlv_pose_cb(data):
    global tlv_signal
    data.orientation.y = tlv_signal
    v2x_tlv_pose.publish(data)

def hlv_path_cb(data):
    v2x_hlv_path.publish(data)

def tlv_path_cb(data):
    v2x_tlv_path.publish(data)

def hlv_state_cb(data):
    _hlv_system = Float32MultiArray()
    latency = random.randint(1200,4000)
    _hlv_system.data = [float(data.data), 1, 1, latency, 1500,70, 30]
    hlv_system.publish(_hlv_system)

def tlv_state_cb(data):
    _tlv_system = Float32MultiArray()
    latency = random.randint(1200,4000)
    _tlv_system.data = [float(data.data), 1, 1, latency, 1400,80, 45]
    tlv_system.publish(_tlv_system)

def hlv_signal_cb(data):
    global hlv_signal
    hlv_signal = data.data

def tlv_signal_cb(data):
    global tlv_signal
    tlv_signal = data.data

def listener():
    rospy.init_node('fake_v2x', anonymous=True)
    rospy.Subscriber('/car/hlv_pose', Pose, hlv_pose_cb)
    rospy.Subscriber('/car/tlv_pose', Pose, tlv_pose_cb)
    rospy.Subscriber('/planning/hlv_path', Marker, hlv_path_cb)
    rospy.Subscriber('/planning/tlv_path', Marker, tlv_path_cb)
    rospy.Subscriber('/hlv_state', Int8, hlv_state_cb)
    rospy.Subscriber('/tlv_state', Int8, tlv_state_cb)
    rospy.Subscriber('/hlv_signal', Int8, hlv_signal_cb)
    rospy.Subscriber('/tlv_signal', Int8, tlv_signal_cb)
    
    rospy.spin()

if __name__ == '__main__':
    listener()