#!/usr/bin/env python
import rospy
import threading
import signal
import math
import tf

from geometry_msgs.msg import  Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int8, Float32MultiArray

def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT received")


class RosManager:
    def __init__(self, v2v_sharing, type):
        cars = ['', 'hlv', 'tlv']
        colors = [(), (241/255,76/255,152/255, 0.5),(94/255, 204/255, 243/255, 0.5)]
        self.car = cars[int(type)]
        self.target = cars[3-int(type)]
        self.target_color = colors[3-int(type)]
        rospy.init_node(f"v2v_sharing_{self.car}")
        self.v2v_sharing = v2v_sharing
        self.vehicle_path = [[0],[0]]
        self.vehicle_state = [0,0,0,0,0,0]
        self.Hz = 5
        self.rate = rospy.Rate(self.Hz)
        self.shutdown_event = threading.Event()
        

    def set_protocol(self):
        rospy.Subscriber(f'/car/{self.car}_pose',Pose, self.pose_cb)
        rospy.Subscriber(f'/planning/{self.car}_path',Marker, self.path_cb)
        rospy.Subscriber(f'/{self.car}_state',Int8, self.state_cb)
        rospy.Subscriber(f'/{self.car}_signal', Int8, self.signal_cb)

        self.system_pub = rospy.Publisher(f'/{self.car}_system', Float32MultiArray, queue_size=1)
        self.target_pose_pub = rospy.Publisher(f'/v2x/{self.target}_pose', Pose, queue_size=1)
        self.target_path_pub = rospy.Publisher(f'/v2x/{self.target}_path', Marker, queue_size=1)
    
    def pose_cb(self, msg):
        self.vehicle_state[2] = msg.position.x
        self.vehicle_state[3] = msg.position.y
        self.vehicle_state[4] = msg.position.z
        self.vehicle_state[5] = msg.orientation.x
    
    def signal_cb(self, msg):
        self.vehicle_state[1] = msg.data

    def state_cb(self, msg):
        self.vehicle_state[0] = msg.data
    
    def path_cb(self, msg):
        self.vehicle_path = [[],[]]
        for pt in msg.points:
            self.vehicle_path[0].append(pt.x)
            self.vehicle_path[1].append(pt.y)
        if len(self.vehicle_path) == 0:
            self.vehicle_path = [[0], [0]]

    def publish(self, result):
        if result == [0,0]:
            return
        state = result[0]
        path = result[1]
        target_pose = Pose()
        target_pose.position.x = state[2]
        target_pose.position.y = state[3]
        target_pose.position.z = state[4]
        target_pose.orientation.x = state[5]
        target_pose.orientation.y = state[1]

        target_path = Marker()
        target_path.type = Marker.LINE_STRIP
        target_path.action = Marker.ADD
        target_path.header.frame_id = 'world'
        target_path.ns = 'target_path'
        target_path.id = 1
        target_path.text = 'target_path'
        target_path.lifetime = rospy.Duration(0)
        target_path.scale.x = 1.5
        target_path.color.r = self.target_color[0]
        target_path.color.g = self.target_color[1]
        target_path.color.b = self.target_color[2]
        target_path.color.a = self.target_color[3]
        target_path.pose.orientation.x = 0.0
        target_path.pose.orientation.y = 0.0
        target_path.pose.orientation.z = 0.0
        
        target_path.pose.orientation.w = 1.0

        for i, xs in enumerate(path[0]):
            target_path.points.append(Point(x=xs, y=path[1][i], z=0.2))

        self.target_pose_pub.publish(target_pose)
        self.target_path_pub.publish(target_path)
    
    def publish_calc(self, system):
        self.system_pub.publish(Float32MultiArray(data=list(system.values())))

    def do_tx(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            tx_res = self.v2v_sharing.do_tx(self.vehicle_state, self.vehicle_path)
            if tx_res<0:
                rospy.logerr("[V2X ROSManager] Tx Send Data Failed")
                return -1
            self.rate.sleep()
    
    def do_rx(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            rx_res = self.v2v_sharing.do_rx()
            print(rx_res)
            if rx_res == None:
                rospy.logerr("[V2X ROSManager] No Rx Data from Target")
                return -1
            else:
                self.publish(rx_res)
            self.rate.sleep()
    
    def do_calc_rate(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            calc_rates_res = self.v2v_sharing.do_calc_rate(self.Hz)
            if calc_rates_res < 0:
                rospy.logerr("[V2X ROSManager] Rx Not Working")
                rate.sleep()
                continue
            rate.sleep()

    def do_calc(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            calc_res = self.v2v_sharing.do_calc()
            if calc_res == -1:
                pass
            else:
                self.publish_calc(calc_res)
            self.rate.sleep()
            

    def execute(self):
        signal.signal(signal.SIGINT, signal_handler)
        self.info_received = False
        self.set_protocol()

        if self.v2v_sharing.set_obu() < 0 :
            return -1
        
        sharing_state = 1

        thread1 = threading.Thread(target=self.do_tx)
        thread2 = threading.Thread(target=self.do_rx)
        thread3 = threading.Thread(target=self.do_calc)
        thread4 = threading.Thread(target=self.do_calc_rate)

        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()

        try:
            thread1.join()
            thread2.join()
            thread3.join()
            thread4.join()
        except KeyboardInterrupt:
            self.shutdown_event.set()
            thread1.join()
            thread2.join()
            thread3.join()
            thread4.join()
            

        return sharing_state