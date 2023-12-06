#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time
import os
import sys
import signal
from tabulate import tabulate

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3, Pose
from novatel_oem7_msgs.msg import INSPVA

over = False
class IONIQ5():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('./dbc_files/ioniq.dbc')

        self.control_state = {'pa_enable': 0x0, 'lon_enable': 0x0 }
        self.target_actuators = {'steer': 0, 'accel': 0, 'brake': 0}
        self.signal = {'left':0, 'right': 0}
        self.tick = {0.01: 0, 0.02: 0, 0.1: 0, 0.2: 0, 0.5: 0, 0.09: 0, 1: 0}
        self.Accel_Override = 0
        self.Break_Override = 0
        self.Steering_Overide = 0
        self.PA_Enable_Status = 0
        self.LON_Enable_Status = 0
        self.alv_cnt = 0
        self.car_mode = 0
        self.user_mode = 0
        self.reset = 0
        self.saver_time = time.time()

        self.pose = Pose() 
        self.pub_mode = rospy.Publisher('/car/mode', Int8, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/hlv_pose', Pose, queue_size=1)
        rospy.Subscriber('/selfdrive/hlv_actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/mode', Int8, self.user_mode_cb)
        rospy.Subscriber('/hlv_signal', Int8, self.signal_cb)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)

        if not os.path.exists('./log/'):
            os.makedirs('./log/')
        self.path = f'./log/{time.time()}.txt'

    def reset_trigger(self):
        if self.Accel_Override or self.Break_Override or self.Steering_Overide:
            self.reset = 1
        elif self.reset and (self.Accel_Override and self.Break_Override and self.Steering_Overide) == 0:
            self.reset = 0

    def user_mode_cb(self, msg):
        self.user_mode = msg.data
    
    def mode_receiver(self):
        state = self.control_state
        if self.user_mode == 0:  
            self.reset_trigger()
            state = {**state, 'pa_enable': 0x0, 'lon_enable': 0x0}
        elif self.user_mode == 1:  # Full
            state = {**state, 'pa_enable': 0x1, 'lon_enable': 0x1}
            self.Accel_Override = 0
            self.Break_Override = 0
            self.Steering_Overide = 0 
        elif self.user_mode == 2:  # Only Lateral
            state = {**state, 'pa_enable': 0x1, 'lon_enable': 0x0}
        elif self.user_mode == 3:  # Only Longitudinal
            state = {**state, 'pa_enable': 0x0, 'lon_enable': 0x1}

        # if any((self.Accel_Override, self.Break_Override, self.Steering_Overide)):
        #     state = {**state, 'pa_enable': 0x0, 'lon_enable': 0x0}
        #     self.reset_trigger()
        self.control_state = state 
    
    def signal_cb(self, msg):
        if msg.data == 1:
            self.signal = {**self.signal, 'left':1, 'right':0}
        elif msg.data == 2:
            self.signal = {**self.signal, 'left':0, 'right':1}
        else:
            self.signal = {**self.signal, 'left':0, 'right':0}
    
    def actuator_cb(self, msg):
        if self.car_mode == 1:
            #self.target_actuators = {**self.target_actuators, 'steer':msg.x, 'accel':msg.y, 'brake':msg.z}
            self.target_actuators['steer'] = msg.x 
            self.target_actuators['accel'] = msg.y
            self.target_actuators['brake'] = msg.z
        else:
            self.target_actuators = {**self.target_actuators, 'steer':0, 'accel':0, 'brake':0}

    def novatel_cb(self, msg):
        self.pose.position.x = msg.latitude
        self.pose.position.y = msg.longitude
        self.pose.position.z = 89 - msg.azimuth  

    def receiver(self):
        data = self.bus.recv(0.2)
        try:
            if (data.arbitration_id == 0x280):
                res = self.db.decode_message(data.arbitration_id, data.data)
                RL = res['Gway_Wheel_Velocity_RL']
                RR = res['Gway_Wheel_Velocity_RR']
                self.pose.orientation.x = (RR + RL)/7.2
            if(data.arbitration_id == 656):
                res = self.db.decode_message(data.arbitration_id, data.data)
                steer = res['Gway_Steering_Angle']
                self.pose.orientation.y = steer
            if (data.arbitration_id == 368):
                res = self.db.decode_message(data.arbitration_id, data.data)
                accel = res['Gway_Accel_Pedal_Position']
                self.pose.orientation.z = accel
            if (data.arbitration_id == 304):
                res = self.db.decode_message(data.arbitration_id, data.data)
                brake= res['Gway_Brake_Cylinder_Pressure']
                self.pose.orientation.w = brake
            if (data.arbitration_id == 784):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.Accel_Override = res['Accel_Override']
                self.Break_Override = res['Break_Override']
                self.Steering_Overide = res['Steering_Overide']
            if (data.arbitration_id == 529):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.PA_Enable_Status = res['PA_Enable_Status']
                self.LON_Enable_Status = res['LON_Enable_Status']
                self.check_mode()
        except Exception as e:
            print(e)
    
    def check_mode(self):
        if self.PA_Enable_Status == 0 and self.LON_Enable_Status == 0:
            self.car_mode = 0
        elif self.PA_Enable_Status == 1 and self.LON_Enable_Status == 1:
            self.car_mode = 1
        elif self.PA_Enable_Status == 1 and self.LON_Enable_Status == 0:
            self.car_mode = 2
        elif self.PA_Enable_Status == 0 and self.LON_Enable_Status == 1:
            self.car_mode = 3

    def sender(self):
        self.alv_cnt = (self.alv_cnt + 1) % 256
        signals = {'PA_Enable': self.control_state['pa_enable'], 'PA_StrAngCmd': self.target_actuators['steer'],
                   'LON_Enable': self.control_state['lon_enable'], 'Target_Brake': self.target_actuators['brake'], 
                   'Target_Accel': self.target_actuators['accel'], 'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset,
                   'TURN_SIG_LEFT': self.signal['left'], 'TURN_SIG_RIGHT':self.signal['right']}
        msg = self.db.encode_message('Control', signals)
        can_msg = can.Message(arbitration_id=0x210,data=msg, is_extended_id=False)
        self.bus.send(can_msg)
    
    def publisher(self):
        self.pub_pose.publish(self.pose)
        self.pub_mode.publish(Int8(self.car_mode))
    
    def checker(self):
        print("Enable & Override Status")
        data = [
            ["PA Enable", "LON Enable", "Accel Override", "Brake Override", "Steering Override"],
            [self.PA_Enable_Status, self.LON_Enable_Status, self.Accel_Override, self.Break_Override, self.Steering_Overide]
        ]
        print(tabulate(data, tablefmt="grid"))
        print("Ego Pose")
        data = [
            ["Latitude","Longitude", "Heading", "Velocity", "Accel", "Brake", "Steer"],
            [f"{self.pose.position.x:.4f}", f"{self.pose.position.y:.4f}", f"{self.pose.position.z:.2f}", f"{self.pose.orientation.x:3f}", f"{self.pose.orientation.z:3f}", f"{self.pose.orientation.w:3f}", f"{self.pose.orientation.y:2f}"]
        ]
        print(tabulate(data,  tablefmt="grid"))

        print("Target Controls")
        data = [
            ["PA Enable", "LON Enable", "Accel", "Brake", "Steer", "L Signal", "R Signal", "Alive", "Reset"],
            [self.control_state['pa_enable'], self.control_state['lon_enable'], self.target_actuators['accel'], self.target_actuators['brake'],
             self.target_actuators['steer'], self.signal['left'], self.signal['right'], self.alv_cnt, self.reset]
        ]
        print(tabulate(data, tablefmt="grid"))
    
    def saver(self):
        elapsed = time.time()-self.saver_time
        data = f"{elapsed:.2f} {self.pose.orientation.x} {self.pose.orientation.z} {self.pose.orientation.w}\n"
        with open(self.path, 'a') as f:
            f.writelines(data)
        self.saver_time = time.time()

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False

    def cleanup(self):
        self.bus.shutdown()

    def run(self):
        while not rospy.is_shutdown():
            if self.timer(0.02):
                self.sender()
            if self.timer(0.1):
                self.publisher()
                # self.checker()
                # self.saver()
                self.mode_receiver()
            self.receiver()
            if over:
                print("breake")
                break
        rospy.on_shutdown(self.cleanup)

def signal_handler(sig, frame):
    over = True 
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('IONIQ5', anonymous=False)
    i5 = IONIQ5()
    i5.run()
    

if __name__ == "__main__":
    main()
