import os
import json
from datetime import datetime
import random
from tqdm import tqdm

class DuplicateJSON:
    def __init__(self):
        self.txt_file = '/home/kana/Documents/Dataset/OBIGO/SIM/log.txt'
        self.target_path = '/home/kana/Documents/Dataset/OBIGO/SIM/0905'
        self.spend_time = {}

    def folder_maker(self):
        with open(self.txt_file, 'r') as file:
            lines = file.readlines()

        for line in lines:
            splitted = line.split()
            date = splitted[0]
            time_str = splitted[1]
            start_time = splitted[2]
            
            folder = f'/home/kana/Documents/Dataset/OBIGO/SIM/{date}'
            hlv_folder = os.path.join(folder, 'hlv')
            tlv_folder = os.path.join(folder, 'tlv')
            if not os.path.exists(folder):
                os.makedirs(folder)
            if not os.path.exists(hlv_folder):
                os.makedirs(hlv_folder)
            if not os.path.exists(tlv_folder):
                os.makedirs(tlv_folder)

            hours, minutes = map(int, time_str.split(':'))
            self.spend_time[date] = ( hours * 60 + minutes, start_time)

    def duplicate_json(self):
        for seq in self.spend_time.items():
            self.process_json(seq)

    def get_timestamp(self, file_path):
        with open(file_path, 'r') as file:
            data = json.load(file)
        timestamp_str = data['timestamp']
        return datetime.strptime(timestamp_str, "%Y.%m.%d.%H:%M:%S.%f")

    def get_s_time_from_mod(self, mod):
        print(mod)
        splited = mod.split('.')[3]
        ssplited = splited.split(':')
        return f'{ssplited[0]}:{ssplited[1]}'

    def process_json(self, seq): #seq = {'date':spend start_time}
        time_difference = None
        print(seq)
        for v in ['hlv', 'tlv']:
            print(v)
            source_folder = f'{self.target_path}/{v}/'
            destination_folder = f'/home/kana/Documents/Dataset/OBIGO/SIM/{seq[0]}/{v}'
            json_files = [f for f in os.listdir(source_folder) if f.endswith('.json')]
            json_files.sort(key=lambda x: self.get_timestamp(os.path.join(source_folder, x)))

            cnt = 0
            round = 0
            sequence_number = (((seq[1][0])*60) * 2)*2 + random.randint(100, 3000)
            mod_timestamp = ''
            seq_over = False

            while not seq_over:
                print("round :", round)
                round += 1
                stime = seq[1][1] if cnt == 0 else self.get_s_time_from_mod(mod_timestamp)
                
                for idx, json_file in tqdm(enumerate(json_files)):
                    cnt += 1
                    source_path = os.path.join(source_folder, json_file)
                    x_type = json_file.split('_')[0]

                    mod_timestamp, init_time_diff = self.get_modified_timestamp(source_path, idx, seq[0], stime, time_difference)
                    if idx == 0:
                        time_difference = init_time_diff
                    
                    if x_type == 'tx':
                        new_data = self.modify_tx_json(source_path, mod_timestamp)
                    else :
                        new_data = self.modify_rx_json(source_path, mod_timestamp)
                    
                    date_object = datetime.strptime(mod_timestamp, "%Y.%m.%d.%H:%M:%S.%f")
                    new_file_name = int(date_object.timestamp() * 1000)
                    destination_path = f'{destination_folder}/{x_type}_{new_file_name}.json'
                    with open(destination_path, 'w', encoding='utf-8') as file:
                        json.dump(new_data, file, indent=2, ensure_ascii=False)
                    
                    if cnt > sequence_number:
                        seq_over = True
                        break

    def get_modified_timestamp(self, source_path, idx, date, stime, time_difference):
        with open(source_path, 'r') as file:
            data = json.load(file)

        if idx == 0:
            start_time = datetime.strptime(data['timestamp'], "%Y.%m.%d.%H:%M:%S.%f")
            month= int(date[:2])
            day = int(date[2:])
            hour = int((stime).split(':')[0])
            minute = int((stime).split(':')[1])
            sec = random.randint(0, 60)
            millisec = random.randint(0, 999999)
            new_start_time = datetime(2023, month, day, hour, minute, sec, millisec)
            init_time_difference = new_start_time - start_time
            timestamp = f'{2023}.{month}.{day}.{hour}:{minute}:{sec}.{millisec}'

        else:
            timestamp = datetime.strptime(data['timestamp'], "%Y.%m.%d.%H:%M:%S.%f")
            adjusted_timestamp = timestamp + time_difference
            timestamp = adjusted_timestamp.strftime("%Y.%m.%d.%H:%M:%S.%f")
            init_time_difference = time_difference
        
        random_milliseconds = random.randint(0, 999)
        timestamp = f"{timestamp[:-3]}{random_milliseconds:03}"
        return timestamp, init_time_difference

    def random_value(self, val):
        decimal_part_str = str(val).split('.')[1]
        new_decimal_part_str = ''.join(random.choices('0123456789', k=len(decimal_part_str)))
        new_number = float(f"{val:.{len(decimal_part_str)}f}".replace(decimal_part_str, new_decimal_part_str))
        return new_number

    def modify_tx_json(self, src, mod_timestamp):
        with open(src, 'r', encoding='utf-8', errors='ignore') as file:
            new_data = json.load(file)

            old_path = new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['path']
            old_lat = new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['lat']
            old_long = new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['long']
            old_heading = new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['heading']
            old_velocity = new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['velocity']

            if len(old_path) > 5:
                num_items_to_remove = random.randint(0, len(old_path) - 5)
                items_to_remove = random.sample(range(len(old_path)), num_items_to_remove)
                items_to_remove.sort(reverse=True)  # 뒤에서부터 제거해야 인덱스가 꼬이지 않음
                for index in items_to_remove:
                    del old_path[index]
            new_path = old_path

            new_data['timestamp'] = mod_timestamp
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['length'] = 584 + (len(new_path)*16)
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['ulPayloadLength'] = 520 + (len(new_path)*16)
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['lat'] = self.random_value(old_lat)
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['long'] = self.random_value(old_long)
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['heading'] = self.random_value(old_heading)
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['velocity'] = self.random_value(old_velocity)
            new_data['Ext_V2X_Tx_PDU']['v2x_msg']['data']['path'] = new_path

            return new_data
    
    def modify_rx_json(self, src, mod_timestamp):
        with open(src, 'r', encoding='utf-8', errors='ignore') as file:
            new_data = json.load(file)

            old_path = new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['path']
            old_lat = new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['lat']
            old_long = new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['long']
            old_heading = new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['heading']
            old_velocity = new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['velocity']

            if len(old_path) > 5:
                num_items_to_remove = random.randint(0, len(old_path) - 5)
                items_to_remove = random.sample(range(len(old_path)), num_items_to_remove)
                items_to_remove.sort(reverse=True)  # 뒤에서부터 제거해야 인덱스가 꼬이지 않음
                for index in items_to_remove:
                    del old_path[index]
            new_path = old_path

            new_data['timestamp'] = mod_timestamp
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['length'] = 584 + (len(new_path)*16)
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['ulPayloadLength'] = 520 + (len(new_path)*16)
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['lat'] = self.random_value(old_lat)
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['long'] = self.random_value(old_long)
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['heading'] = self.random_value(old_heading)
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['messageFrame']['coreData']['velocity'] = self.random_value(old_velocity)
            new_data['Ext_V2X_Rx_PDU']['v2x_msg']['data']['path'] = new_path

            return new_data

        

if __name__ == "__main__":
    dj = DuplicateJSON()
    dj.folder_maker()
    dj.duplicate_json()

