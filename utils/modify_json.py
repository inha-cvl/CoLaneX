import json
import os
from tqdm import tqdm
import random

def modify_json_date(file_path):
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        data = json.load(file)
    old_ts = data['timestamp']
    tp = old_ts.split(':')
    tpd = (tp[0]).split('.')
    if len(tpd) == 2:
        tpd[0] = 2023
        d = int(tpd[1])-10
        tpd[1] = 10
        tpd.append(11)
    else:
        d = int(tpd[-1])-10
    tpp = (tp[1]).split('.')
    tpm = tpp[1]
    s = tpm[:2]
    ms = tpm[2:5]
    new_ts = f'{tpd[0]}.{tpd[1]}.{tpd[2]}.{d}:{tpp[0]}:{s}.{ms}'
    data['timestamp'] = new_ts
    with open(file_path, 'w', encoding='utf-8') as file:
        json.dump(data, file, indent=2, ensure_ascii=False)

for v in ['hlv', 'tlv']:
    directory = f"/home/kana/Documents/Dataset/OBIGO/SIM/1011/{v}" 
    for filename in tqdm(os.listdir(directory)):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            modify_json_date(file_path)

'''
def modify_json_file(file_path):
    # 파일 읽기
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        data = json.load(file)

    timestamp_parts = data["timestamp"].split('.')
    last_number = int(timestamp_parts[-1])
    random_subtract = random.randint(4000, 5000)
    modified_number = last_number + random_subtract
    timestamp_parts[-1] = str(modified_number)
    modified_timestamp = '.'.join(timestamp_parts)
    timestamp = modified_timestamp

    length = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["length"]
    ulPayloadLength = length-64 if length > 64 else 0
    path =  data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["path"]
    msgCnt =  int(data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["rx_msg_cnt"])+2
    rx_msg_cnt = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["msgCnt"]
    lat = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["lat"]
    long = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["long"]
    signal = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["signal"]
    heading = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["heading"]
    velocity = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["velocity"]
    
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
                            "msgCnt":msgCnt,
                            "rx_msg_cnt":rx_msg_cnt,
                            "lat": lat,
                            "long":long,
                            "signal":signal,
                            "heading":heading,
                            "velocity":velocity,   
                        }
                    },
                    "path":path
                }
            }
        }
    }
        
    # 수정된 데이터를 파일에 쓰기
    filename_ts_parts = file_path.split('_')[1]
    filename_ts_s_parts = int(filename_ts_parts.split('.')[0])
    file_name_ts = filename_ts_s_parts+4
    new_path = f"/home/kana/Documents/Dataset/OBIGO/SIM/1020/tlv/tx_{file_name_ts}.json"
    with open(new_path, 'w', encoding='utf-8') as file:
        json.dump(new_data, file, indent=2, ensure_ascii=False)

# "rx_"로 시작하는 파일들에 대해 작업 수행
directory = "/home/kana/Documents/Dataset/OBIGO/SIM/1020/hlv"  # 디렉토리 경로를 적절히 수정하세요.

for filename in tqdm(os.listdir(directory)):
    if filename.startswith("rx_") and filename.endswith(".json"):
        file_path = os.path.join(directory, filename)
        modify_json_file(file_path)

'''
'''
def modify_json_file(file_path):
    # 파일 읽기
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        data = json.load(file)

    timestamp_parts = data["timestamp"].split('.')
    last_number = int(timestamp_parts[-1])
    random_subtract = random.randint(4000, 5000)
    modified_number = last_number - random_subtract
    timestamp_parts[-1] = str(modified_number)
    modified_timestamp = '.'.join(timestamp_parts)
    timestamp = modified_timestamp

    length = data["Ext_V2X_Tx_PDU"]["v2x_msg"]["length"]
    ulPayloadLength = length-64 if length > 64 else 0
    path =  data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["path"]

    msgCnt =  data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["rx_msg_cnt"]
    rx_msg_cnt = int(data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["msgCnt"])-2
    lat = data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["lat"]
    long = data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["long"]
    signal = data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["signal"]
    heading = data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["heading"]
    velocity = data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["messageFrame"]["coreData"]["velocity"]
    
    new_data = {
        "timestamp":timestamp, 
        "Ext_V2X_Rx_PDU": {
            "magic_num": 62194,
            "ver": 1,
            "psid": 5271,
            "e_v2x_comm_type": 3,
            "e_payload_type": 0,
            "freq": 59670,
            "rssi": 0,
            "is_signed": 0,
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
                            "msgCnt":msgCnt,
                            "tx_msg_cnt":rx_msg_cnt,
                            "lat": lat,
                            "long":long,
                            "signal":signal,
                            "heading":heading,
                            "velocity":velocity,   
                        }
                    },
                    "path":path
                }
            }
        }
    }
        
    # 수정된 데이터를 파일에 쓰기
    filename_ts_parts = file_path.split('_')[1]
    filename_ts_s_parts = int(filename_ts_parts.split('.')[0])
    file_name_ts = filename_ts_s_parts-4
    new_path = f"./log/tlv/rx_{file_name_ts}.json"
    with open(new_path, 'w', encoding='utf-8') as file:
        json.dump(new_data, file, indent=2, ensure_ascii=False)

# "rx_"로 시작하는 파일들에 대해 작업 수행
directory = "./log/hlv/"  # 디렉토리 경로를 적절히 수정하세요.

for filename in os.listdir(directory):
    if filename.startswith("tx_") and filename.endswith(".json"):
        file_path = os.path.join(directory, filename)
        print(file_path)
        modify_json_file(file_path)

'''        




'''
import json
import os
def modify_json_file(file_path):
    # 파일 읽기
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        data = json.load(file)
    # 값 수정
    data["Ext_V2X_Rx_PDU"]["psid"] = 5272
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["eDeviceType"] = 1
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["eTeleCommType"] = 30
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["unDeviceId"] = 72
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["ulTimeStamp"] = 0
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["eServiceId"] = 4
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["eActionType"] = 1
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["eRegionId"] = 1
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["ePayloadType"] = 1
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["eCommId"] = 1

    # ulPayloadLength 수정
    length = data["Ext_V2X_Rx_PDU"]["v2x_msg"]["length"]
    data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["ulPayloadLength"] = length - 64
    # 수정된 데이터를 파일에 쓰기
    with open(file_path, 'w', encoding='utf-8') as file:
        json.dump(data, file, indent=2, ensure_ascii=False)

# "rx_"로 시작하는 파일들에 대해 작업 수행
directory = "./log/hlv/"  # 디렉토리 경로를 적절히 수정하세요.

for filename in os.listdir(directory):
    if filename.startswith("x_") and filename.endswith(".json"):
        file_path = os.path.join(directory, filename)
        print(file_path)
        modify_json_file(file_path)
'''