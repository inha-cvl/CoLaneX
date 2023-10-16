import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import paho.mqtt.client as mqtt
import json


broker_address = "infocabin.obigo.com"
broker_port = 1883

def on_disconnect(client, userdata, flags, rc=0):
    print(str(rc))


def on_publish(client, userdata, mid):
    print("In on_pub callback mid= ", mid)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)

def hlv_system(msg):
    #print(msg)
    client.publish('/hlv_system', str(msg))

def tlv_system(msg):
    client.publish('/tlv_system', str(msg))

def hlv_geojson(msg):
    client.publish('/planning/hlv_geojson', str(msg))

def tlv_geojson(msg):
    client.publish('/planning/tlv_geojson', str(msg))

def hlv_pose(msg):
    client.publish('/car/hlv_pose', str(msg))

def tlv_pose(msg):
    client.publish('/car/tlv_pose', str(msg))

if __name__ == "__main__":
    rospy.init_node("sub_test")

    client = mqtt.Client()

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    client.connect(broker_address, broker_port)
    client.loop_start()
    #client.loop_stop()
    rospy.Subscriber("/hlv_system", Float32MultiArray, hlv_system)
    rospy.Subscriber('/tlv_system', Float32MultiArray, tlv_system)
    rospy.Subscriber('/planning/hlv_geojson', String, hlv_geojson)
    rospy.Subscriber('/planning/tlv_geojson', String, tlv_geojson)
    rospy.Subscriber('/car/hlv_pose', Pose, hlv_pose)
    rospy.Subscriber('/car/tlv_pose', Pose, tlv_pose)

    rospy.spin()
