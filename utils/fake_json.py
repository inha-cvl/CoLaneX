import rospy
import random
import pymap3d as pm
import math
import time
import json
from datetime import datetime
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


class FakeJSON:
    def __init__(self):
        rospy.init_node('fake_json', anonymous=True)

        self.dataset_path = '/home/kana/Documents/Dataset/OBIGO/SIM/0906'
        self.round = ((  180 * 60 ) * 2)
        self.max_packet =( self.round * 5 )

        self.hlv_signal = 0
        self.tlv_signal = 0

        self.hlv_tx_msg_cnt = 0
        self.hlv_tx_rx_msg_cnt = 0
        self.hlv_rx_msg_cnt = 0
        self.hlv_rx_tx_msg_cnt = 0

        self.tlv_tx_msg_cnt = 0
        self.tlv_tx_rx_msg_cnt = 0
        self.tlv_rx_msg_cnt = 0
        self.tlv_rx_tx_msg_cnt = 0

        self.hlv_path = []
        self.tlv_path = []

        self.hlv_tx_random_idx = random.sample(range(self.max_packet), self.max_packet-self.round)
        self.hlv_rx_random_idx = random.sample(range(self.max_packet), self.max_packet-self.round)
        self.tlv_tx_random_idx = random.sample(range(self.max_packet), self.max_packet-self.round)
        self.tlv_rx_random_idx = random.sample(range(self.max_packet), self.max_packet-self.round)


    
    def random_position(self, val):
        decimal_part_str = str(val).split('.')[1]
        new_decimal_part_str = ''.join(random.choices('0123456789', k=len(decimal_part_str)))
        new_number = float(f"{val:.{len(decimal_part_str)}f}".replace(decimal_part_str, new_decimal_part_str))
        return new_number

    def hlv_pose_cb(self, data):
        self.hlv_tx_msg_cnt += 1
        if self.hlv_tx_msg_cnt > self.max_packet:
            print("FINISH")
            return
        
        current_timestamp  = datetime.now()
        timestamp= current_timestamp.strftime('%Y.%m.%d.%H:%M:%S.%f')[:-3]
        timer = time.time()
        t = time.localtime(timer)
        sec_in_mill = int(time.mktime(t) * 1000 + current_timestamp.microsecond / 1000)-random.randint(0, 999)
        
        length = 584 + (len(self.hlv_path)*16)
        ulPayloadLength = length-64

        self.hlv_tx_rx_msg_cnt = self.hlv_rx_msg_cnt -random.randint(0,2) if self.hlv_rx_msg_cnt > 0 else self.hlv_rx_msg_cnt

        self.hlv_timestamp = timestamp
        self.hlv_v2x_msg_length = length
        self.hlv_v2x_msg_ulPayloadLength = ulPayloadLength
        self.hlv_v2x_msg_msgCnt = self.hlv_tx_msg_cnt
        self.hlv_v2x_msg_rx_msg_cnt = self.hlv_tx_rx_msg_cnt
        self.hlv_v2x_msg_lat = self.random_position(data.position.x*math.pow(10, 7))
        self.hlv_v2x_msg_long = self.random_position(data.position.y*math.pow(10,7))
        self.hlv_v2x_msg_signal = self.hlv_signal,
        self.hlv_v2x_msg_heading = self.random_position(data.position.z/0.0125)
        self.hlv_v2x_msg_velocity= self.random_position(data.orientation.x/0.02)
        self.hlv_v2x_msg_path = self.hlv_path
        
        #Tx
        new_data = {
            "timestamp":timestamp, 
            "Ext_V2X_Tx_PDU": {
                "ver": 1,
                "e_payload_type": 0,
                "psid": 5271,
                "tx_power": 20,
                "e_signer_id": 0,
                "e_priority": 0,
                "magic_num": 62194,#62450,
                "u.config_cv2x": {
                    "transmitter_profile_id": 100,
                    "peer_l2id": 0
                },
                "v2x_msg": {
                    "length": length,
                    "data": {
                        "eDeviceType": 1,
                        "eTeleCommType": 30,
                        "unDeviceId": 71,
                        "ulTimeStamp": 0,
                        "eServiceId": 4,
                        "eActionType": 1,
                        "eRegionId": 1,
                        "ePayloadType": 1,
                        "eCommId": 1,
                        "ulPayloadLength": ulPayloadLength,
                        "messageFrame":{
                            "coreData":{
                                "msgCnt":self.hlv_tx_msg_cnt,
                                "rx_msg_cnt":self.hlv_tx_rx_msg_cnt,
                                "lat": self.hlv_v2x_msg_lat ,
                                "long":self.hlv_v2x_msg_long ,
                                "signal":self.hlv_signal,
                                "heading":self.hlv_v2x_msg_heading,
                                "velocity":self.hlv_v2x_msg_velocity,   
                            }
                        },
                        "path":self.hlv_path
                    }
                }
            }
        }
        
        if self.hlv_tx_msg_cnt not in self.hlv_tx_random_idx:
            hlv_tx_path = f"{self.dataset_path}/hlv/tx_{sec_in_mill}.json"
            with open(hlv_tx_path, 'w', encoding='utf-8') as file:
                json.dump(new_data, file, indent=2, ensure_ascii=False)

    def tlv_pose_cb(self, data):
        self.tlv_tx_msg_cnt += 1
        if self.tlv_tx_msg_cnt > self.max_packet:
            return
        
        current_timestamp  = datetime.now()
        timestamp= current_timestamp.strftime('%Y.%m.%d.%H:%M:%S.%f')[:-3]
        timer = time.time()
        t = time.localtime(timer)
        sec_in_mill = int(time.mktime(t) * 1000 + current_timestamp.microsecond / 1000)-random.randint(0, 999)
        
        self.tlv_tx_rx_msg_cnt = self.tlv_rx_msg_cnt -random.randint(0,2) if self.tlv_rx_msg_cnt > 0 else self.tlv_rx_msg_cnt

        length = 584 + (len(self.tlv_path)*16)
        ulPayloadLength = length-64

        self.tlv_timestamp = timestamp
        self.tlv_v2x_msg_length = length
        self.tlv_v2x_msg_ulPayloadLength = ulPayloadLength
        self.tlv_v2x_msg_msgCnt = self.tlv_tx_msg_cnt
        self.tlv_v2x_msg_rx_msg_cnt = self.tlv_tx_rx_msg_cnt
        self.tlv_v2x_msg_lat =  self.random_position(data.position.x*math.pow(10, 7)) 
        self.tlv_v2x_msg_long =  self.random_position(data.position.y*math.pow(10,7))
        self.tlv_v2x_msg_signal = self.tlv_signal,
        self.tlv_v2x_msg_heading = self.random_position(data.position.z/0.0125)
        self.tlv_v2x_msg_velocity= self.random_position(data.orientation.x/0.02)
        self.tlv_v2x_msg_path = self.tlv_path

        #Tx
        new_data = {
            "timestamp":timestamp, 
            "Ext_V2X_Tx_PDU": {
                "ver": 1,
                "e_payload_type": 0,
                "psid": 5272,
                "tx_power": 20,
                "e_signer_id": 0,
                "e_priority": 0,
                "magic_num": 62450,
                "u.config_cv2x": {
                    "transmitter_profile_id": 100,
                    "peer_l2id": 0
                },
                "v2x_msg": {
                    "length": length,
                    "data": {
                        "eDeviceType": 1,
                        "eTeleCommType": 30,
                        "unDeviceId": 72,
                        "ulTimeStamp": 0,
                        "eServiceId": 4,
                        "eActionType": 1,
                        "eRegionId": 1,
                        "ePayloadType": 1,
                        "eCommId": 1,
                        "ulPayloadLength": ulPayloadLength,
                        "messageFrame":{
                            "coreData":{
                                "msgCnt":self.tlv_tx_msg_cnt,
                                "rx_msg_cnt":self.tlv_tx_rx_msg_cnt,
                                "lat": self.tlv_v2x_msg_lat,
                                "long":self.tlv_v2x_msg_long,
                                "signal":self.tlv_signal,
                                "heading":self.tlv_v2x_msg_heading,
                                "velocity":self.tlv_v2x_msg_velocity,   
                            }
                        },
                        "path":self.tlv_path
                    }
                }
            }
        }
        
        if self.tlv_tx_msg_cnt not in self.tlv_tx_random_idx:
            tlv_tx_path = f"{self.dataset_path}/tlv/tx_{sec_in_mill}.json"
            with open(tlv_tx_path, 'w', encoding='utf-8') as file:
                json.dump(new_data, file, indent=2, ensure_ascii=False)
        
    def hlv_path_cb(self, data):
        self.hlv_path = [{"x":pt.x, "y":pt.y} for pt in data.points]
        self.hlv_rx_msg_cnt += 1
        if self.hlv_rx_msg_cnt > self.max_packet:
            return

        current_timestamp  = datetime.now()
        timestamp= current_timestamp.strftime('%Y.%m.%d.%H:%M:%S.%f')[:-3]
        timer = time.time()
        t = time.localtime(timer)
        sec_in_mill = int(time.mktime(t) * 1000 + current_timestamp.microsecond / 1000)-random.randint(0, 999)-random.randint(0, 999)

        self.hlv_rx_tx_msg_cnt = self.hlv_tx_msg_cnt -random.randint(0,2) if self.hlv_tx_msg_cnt > 0 else self.hlv_tx_msg_cnt

        new_data = {
            "timestamp":timestamp, 
            "Ext_V2X_Rx_PDU": {
                "magic_num": 62450,
                "ver": 1,
                "psid": 5272,
                "e_v2x_comm_type": 3,
                "e_payload_type": 0,
                "freq": 59670,
                "rssi": 0,
                "is_signed": 0,
                "v2x_msg": {
                    "length": self.tlv_v2x_msg_length,
                    "data": {
                        "eDeviceType": 1,
                        "eTeleCommType": 30,
                        "unDeviceId": 72,
                        "ulTimeStamp": 0,
                        "eServiceId": 4,
                        "eActionType": 1,
                        "eRegionId": 1,
                        "ePayloadType": 1,
                        "eCommId": 1,
                        "ulPayloadLength": self.tlv_v2x_msg_ulPayloadLength,
                        "messageFrame":{
                            "coreData":{
                                "msgCnt":self.tlv_tx_msg_cnt,
                                "tx_msg_cnt":self.hlv_rx_tx_msg_cnt,
                                "lat": self.tlv_v2x_msg_lat,
                                "long":self.tlv_v2x_msg_long,
                                "signal":self.tlv_v2x_msg_signal,
                                "heading":self.tlv_v2x_msg_heading,
                                "velocity":self.tlv_v2x_msg_velocity,   
                            }
                        },
                        "path":self.tlv_v2x_msg_path
                    }
                }
            }
        }

        if self.hlv_rx_msg_cnt not in self.hlv_rx_random_idx:
            hlv_rx_path = f"{self.dataset_path}/hlv/rx_{sec_in_mill}.json"
            with open(hlv_rx_path, 'w', encoding='utf-8') as file:
                json.dump(new_data, file, indent=2, ensure_ascii=False)


    def tlv_path_cb(self, data):
        self.tlv_path = [{"x":pt.x, "y":pt.y} for pt in data.points]
        self.tlv_rx_msg_cnt += 1
        if self.tlv_rx_msg_cnt > self.max_packet:
            return

        current_timestamp  = datetime.now()
        timestamp= current_timestamp.strftime('%Y.%m.%d.%H:%M:%S.%f')[:-3]
        timer = time.time()
        t = time.localtime(timer)
        sec_in_mill = int(time.mktime(t) * 1000 + current_timestamp.microsecond / 1000)-random.randint(0, 999)

        self.tlv_rx_tx_msg_cnt = self.tlv_tx_msg_cnt -random.randint(0,2) if self.tlv_tx_msg_cnt > 0 else self.tlv_tx_msg_cnt

        new_data = {
            "timestamp":timestamp, 
            "Ext_V2X_Rx_PDU": {
                "magic_num": 62450,
                "ver": 1,
                "psid": 5272,
                "e_v2x_comm_type": 3,
                "e_payload_type": 0,
                "freq": 59670,
                "rssi": 0,
                "is_signed": 0,
                "v2x_msg": {
                    "length": self.hlv_v2x_msg_length,
                    "data": {
                        "eDeviceType": 1,
                        "eTeleCommType": 30,
                        "unDeviceId": 72,
                        "ulTimeStamp": 0,
                        "eServiceId": 4,
                        "eActionType": 1,
                        "eRegionId": 1,
                        "ePayloadType": 1,
                        "eCommId": 1,
                        "ulPayloadLength": self.hlv_v2x_msg_ulPayloadLength,
                        "messageFrame":{
                            "coreData":{
                                "msgCnt":self.hlv_tx_msg_cnt,
                                "tx_msg_cnt":self.tlv_rx_tx_msg_cnt,
                                "lat": self.hlv_v2x_msg_lat,
                                "long":self.hlv_v2x_msg_long,
                                "signal":self.hlv_v2x_msg_signal,
                                "heading":self.hlv_v2x_msg_heading,
                                "velocity":self.hlv_v2x_msg_velocity,   
                            }
                        },
                        "path":self.hlv_v2x_msg_path
                    }
                }
            }
        }

        if self.tlv_rx_msg_cnt not in self.tlv_rx_random_idx:
            tlv_rx_path = f"{self.dataset_path}/tlv/rx_{sec_in_mill}.json"
            with open(tlv_rx_path, 'w', encoding='utf-8') as file:
                json.dump(new_data, file, indent=2, ensure_ascii=False)


    def hlv_state_cb(self, data):
        _hlv_system = Float32MultiArray()
        latency = random.randint(1200,4000)
        _hlv_system.data = [float(data.data), 1, 1, latency, 1500,70, 30]

    def tlv_state_cb(self, data):
        _tlv_system = Float32MultiArray()
        latency = random.randint(1200,4000)
        _tlv_system.data = [float(data.data), 1, 1, latency, 1400,80, 45]

    def hlv_signal_cb(self,data):
        self.hlv_signal = data.data

    def tlv_signal_cb(self, data):
        self.tlv_signal = data.data


    def listener(self):
        
        rospy.Subscriber('/car/hlv_pose', Pose, self.hlv_pose_cb)
        rospy.Subscriber('/car/tlv_pose', Pose, self.tlv_pose_cb)
        rospy.Subscriber('/planning/hlv_path', Marker, self.hlv_path_cb)
        rospy.Subscriber('/planning/tlv_path', Marker, self.tlv_path_cb)
        rospy.Subscriber('/hlv_state', Int8, self.hlv_state_cb)
        rospy.Subscriber('/tlv_state', Int8, self.tlv_state_cb)
        rospy.Subscriber('/hlv_signal', Int8, self.hlv_signal_cb)
        rospy.Subscriber('/tlv_signal', Int8, self.tlv_signal_cb)
        
        rospy.spin()

if __name__ == '__main__':
    fj = FakeJSON()
    fj.listener()