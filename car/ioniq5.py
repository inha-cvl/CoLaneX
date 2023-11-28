#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3, Pose
from novatel_oem7_msgs.msg import INSPVA

class IONIQ5():
    def __init__(self):
        self.control_state = {'steer_en': 0x0, 'acc_en': 0x0 }
        self.target_actuators = {'steer': 0, 'accel': 0, 'brake': 0}
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('./dbc_files/can.dbc')

        self.tick = {0.01: 0, 0.02: 0, 0.2: 0, 0.5: 0, 0.09: 0, 1: 0}
        self.Accel_Override = 0
        self.Break_Override = 0
        self.Steering_Overide = 0
        self.alv_cnt = 0
        self.mode = 0
        self.reset = 0
        self.last_mode_2_time = 0
        self.force_mode_2 = False

        self.pose = Pose() 

        self.pub_mode = rospy.Publisher('/car/mode', Int8, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/hlv_pose', Pose, queue_size=1)
        rospy.Subscriber('/selfdrive/hlv_actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/mode', Int8, self.mode_cb)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_cb)

    def reset_trigger(self):
        if any((self.Accel_Override, self.Break_Override, self.Steering_Overide)):
            self.reset = 1
        elif self.reset and not any((self.Accel_Override, self.Break_Override, self.Steering_Overide)):
            self.reset = 0

    def mode_cb(self, msg):
        state = self.control_state
        current_time = rospy.get_time()
        
        if self.force_mode_2 and current_time - self.last_mode_2_time < 1.0:
            self.mode = 2
        else:
            self.force_mode_2 = False  # 강제 유지를 종료
            if msg.data == 0:  
                self.reset_trigger()
                state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
                if not self.force_mode_2:  
                    self.mode = 0
            elif msg.data == 1:  # Full
                state = {**state, 'steer_en': 0x1, 'acc_en': 0x1}
                self.mode = 1
            elif msg.data == 2:  # Only Lateral
                state = {**state, 'steer_en': 0x1, 'acc_en': 0x0}
            elif msg.data == 3:  # Only Longitudinal
                state = {**state, 'steer_en': 0x0, 'acc_en': 0x1}
            
            if any((self.Accel_Override, self.Break_Override, self.Steering_Overide)):
                self.mode = 2  
                self.last_mode_2_time = current_time  
                self.force_mode_2 = True 
                state = {**state, 'steer_en': 0x0, 'acc_en': 0x0}
                self.reset_trigger()
        
        self.control_state = state 
        
    def actuator_cb(self, msg):
        if self.mode == 1:
            self.target_actuators['steer'] = msg.x
            self.target_actuators['accel'] = msg.y
            self.target_actuators['brake'] = msg.z
        else:
            self.target_actuators['steer'] = 0
            self.target_actuators['accel'] = 0
            self.target_actuators['brake'] = 0

    def novatel_callback(self, msg):
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

            if (data.arbitration_id == 784):
                res = self.db.decode_message(data.arbitration_id, data.data)
                self.Accel_Override = res['Accel_Override']
                self.Break_Override = res['Break_Override']
                self.Steering_Overide = res['Steering_Overide']
        except Exception as e:
            print(e)

    def sender(self):
        self.alv_cnt = (self.alv_cnt + 1) % 256
        signals = {'PA_Enable': self.control_state['steer_en'], 'PA_StrAngCmd': self.target_actuators['steer'],
                   'LON_Enable': self.control_state['acc_en'], 'Target_Brake': self.target_actuators['brake'], 
                   'Target_Accel': self.target_actuators['accel'], 'Alive_cnt': self.alv_cnt, 'Reset_Flag': self.reset}
        msg = self.db.encode_message('Control', signals)
        can_msg = can.Message(arbitration_id=0x210,data=msg, is_extended_id=False)
        self.bus.send(can_msg)
    
    def publisher(self):
        self.pub_pose.publish(self.pose)
        self.pub_mode.publish(Int8(self.mode))

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
                self.publisher()
            self.receiver()
        rospy.on_shutdown(self.cleanup)
    
def main():
    rospy.init_node('IONIQ5', anonymous=False)
    i5 = IONIQ5()
    i5.run()
    

if __name__ == "__main__":
    main()
