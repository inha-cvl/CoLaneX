
import socket
from ctypes import *
from datetime import datetime, timedelta
import struct
import math
import time
import logging

from .v2x_interface import *
from .crc16 import calc_crc16

DEST_PORT = 47347
V2V_PSID = EM_V2V_MSG

class SocketHandler:
    def __init__(self, type, interface, chip):
        self.interface = interface
        self.interface_list = [b'', b'enp4s0', b'enx00e04c6a3d90']
        self.chip = chip
        self.communication_performance = {
            'state': 0,
            'v2x': 0,
            'rtt': 0,
            'mbps': 0,
            'packet_size': 0,
            'packet_rate': 0,
            'distance': 0
        }
        self.rtt_ts_list = []
        self.tx_message = ""
        self.set_logger(type)

    def set_logger(self, type):
        self.logger = logging.getLogger('v2x')
        self.logger.setLevel(logging.DEBUG)
        formatted_datetime = datetime.today().strftime('%m%d%H%M')
        file_handler = logging.FileHandler(f'./log/{self.chip}_{type}_{formatted_datetime}.log')
        file_handler.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        #self.logger.addHandler(console_handler)

    def connect(self, IP):
        try:
            self.fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.interface > 0:
                interface_name = self.interface_list[self.interface]
                try:
                    self.fd.setsockopt(socket.SOL_SOCKET, 25, interface_name)
                except PermissionError:
                    self.logger.error("Permission denied for SO_BINDTODEVICE")
                    return -1
                
            servaddr = socket.getaddrinfo(IP, DEST_PORT, socket.AF_INET, socket.SOCK_STREAM)
            self.fd.connect(servaddr[0][4])
            self.logger.debug("Socket Opend")
            return 1
        except socket.error as e:
            self.logger.error(f"{e}")
            return -1
    
    def register(self):
        buf, hdr = self.get_base()

        hdr.contents.len = socket.htons(SIZE_WSR_DATA-6)
        hdr.contents.seq = 0
        hdr.contents.payload_id = socket.htons(eV2x_App_Payload_Id.ePayloadId_WsmServiceReq)

        wsr = cast(addressof(hdr.contents) + V2x_App_Hdr.data.offset, POINTER(V2x_App_WSR_Add_Crc))
        wsr.contents.action = 0 #add
        wsr.contents.psid = socket.htonl(V2V_PSID) 
        
        _crc16 = calc_crc16(buf[SIZE_MAGIC_NUMBER_OF_HEADER:], SIZE_WSR_DATA - 4 - 2)
        crc16 = pointer(c_uint16.from_buffer(buf, SIZE_WSR_DATA - 2))
        crc16.contents.value = socket.htons(_crc16)
        data = buf[:SIZE_WSR_DATA]
        self.tx_cnt = 0
        self.tx_start_time = time.time()
        return self.send(data, SIZE_WSR_DATA)
    
    def tx(self, state, paths):
        p_overall = self.get_p_overall(self.tx_cnt)
        self.set_tx_values(state)
        size = sizeof(V2x_App_SI_TLVC)
        if self.chip == 'out':
            p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall_V2), POINTER(V2x_App_SI_TLVC))
        else:
            p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall), POINTER(V2x_App_SI_TLVC))
        p_dummy.contents.type = socket.htonl(EM_PT_RAW_DATA)
        p_dummy.contents.len = socket.htons(size + 2)

        p_share_info = cast(addressof(p_dummy.contents.data), POINTER(SharingInformation))
        p_share_info.contents.tx_cnt = socket.htonl(self.tx_cnt)
        p_share_info.contents.tx_cnt_from_rx = socket.htonl(self.tx_cnt_from_rx)
        p_share_info.contents.state = state[0]
        p_share_info.contents.signal = state[1]
        p_share_info.contents.latitude = state[2]
        p_share_info.contents.longitude = state[3]
        p_share_info.contents.heading = state[4]
        p_share_info.contents.velocity = state[5]

        if len(paths) > 0:
            for i, x in enumerate(paths[0]):
                p_share_info.contents.path_x[i] = x
            for i, y in enumerate(paths[1]):
                p_share_info.contents.path_y[i] = y
        
        crc16 = cast(addressof(p_dummy.contents)+size, POINTER(c_uint16))
        crc_data = bytearray(p_dummy.contents)
        crc16.contents.value = socket.htons(calc_crc16(crc_data, size))
        
        package_len = 8 + size
        p_overall.contents.len_package = socket.htons(package_len)

        self.add_ext_status_data(p_overall, package_len)

        if self.chip == 'out':
            self.hdr.contents.len = socket.ntohs(sizeof(V2x_App_Hdr)+6+sizeof(TLVC_Overall_V2)+socket.ntohs(p_overall.contents.len_package))
        else:
            self.hdr.contents.len = socket.ntohs(sizeof(V2x_App_Hdr)+6+sizeof(TLVC_Overall)+socket.ntohs(p_overall.contents.len_package))
        self.hdr.contents.seq = 0
        self.hdr.contents.payload_id = socket.htons(0x10)
        self.tx_msg.contents.psid = socket.htonl(EM_V2V_MSG)
        send_len = socket.ntohs(self.hdr.contents.len) + 6

        crc16 = pointer(c_uint16.from_buffer(self.tx_buf, send_len - 2))
        _crc16 = calc_crc16(self.tx_buf[SIZE_MAGIC_NUMBER_OF_HEADER:], send_len-6)
        crc16.contents.value = socket.htons(_crc16)
        
        data = self.tx_buf[:send_len]
        
        self.communication_performance['packet_size'] = send_len
        self.tx_message = self.get_log_datum(state,paths)

        return self.send(data, send_len)

    def rx(self):
        data = self.receive()
        if data == None:
            return -1
        if len(data) > SIZE_WSR_DATA:
            self.rx_cnt += 1
            self.rx_rate += 1
            hdr_ofs = V2x_App_Hdr.data.offset
            rx_ofs = V2x_App_RxMsg.data.offset
            if self.chip == 'out':
                ovr_ofs = sizeof(TLVC_Overall_V2)
            else:
                ovr_ofs = sizeof(TLVC_Overall)
            tlvc_ofs =  hdr_ofs+rx_ofs+ovr_ofs
            tlvc = V2x_App_SI_TLVC.from_buffer_copy(data,tlvc_ofs)
            sharing_information = tlvc.data
            self.set_rx_values(sharing_information)
            self.tx_cnt_from_rx = socket.ntohl(sharing_information.tx_cnt)
            self.calc_rtt( socket.ntohl(sharing_information.tx_cnt_from_rx))
            
            state, path = self.organize_data(len(data), sharing_information)
            rx_message = self.get_log_datum(state, path)
            self.logger.info(f"Rx cnt:{self.tx_cnt_from_rx}")
            self.logger.info(rx_message)
            return [state, path]
        else:
            return [0, 0]

    def set_tx_values(self, state):
        self.tx_latitude = state[2]
        self.tx_longitude = state[3]
    
    def set_rx_values(self, sharing_information):
        self.rx_latitude = sharing_information.latitude
        self.rx_longtude = sharing_information.longitude

    def calc_rtt(self, tx_cnt_from_rx):
        rtt = 0
        for ts in self.rtt_ts_list:
            if ts[0] == tx_cnt_from_rx:
                rtt = round((time.time()-ts[1])*1000)
                self.communication_performance['rtt'] = rtt
                break
        self.rtt_ts_list[:] = [ts for ts in self.rtt_ts_list if ts[0] >= tx_cnt_from_rx]

    def calc_comm(self):
        if self.tx_cnt < 1 or self.rx_cnt < 1:
            return -1
        else:
            distance = math.sqrt((self.rx_latitude - self.tx_latitude) ** 2 + (self.rx_longtude - self.tx_longitude) ** 2)
            self.communication_performance['v2x'] = 1
            self.communication_performance['distance'] = distance
            self.logger.info(self.get_performance_log())  
            return self.communication_performance        

    def calc_rate(self, hz):
        rx_rate = min(100, (float(self.rx_rate)/hz)*100)
        self.communication_performance['packet_rate'] = rx_rate
        if self.rx_rate >= hz:
            self.rx_rate = 1
        return 1


    def send(self, data, send_size):
        try:
            self.fd.sendall(data)
            self.logger.info(f"Tx cnt:{self.tx_cnt}")
            self.logger.info(self.tx_message)
            self.rtt_ts_list.append([self.tx_cnt, time.time()])
            self.tx_cnt += 1
            current_time = time.time()
            tx_time = current_time - self.tx_start_time
            mbps = (send_size*8/tx_time)/1000000
            self.communication_performance['mbps'] = mbps 
            self.tx_start_time = current_time
            return 1
        except socket.error as e:
            self.logger.error(f"{e}")
            return -1
    
    
    def receive(self):
        try:
            data = self.fd.recv(sizeof(c_char)*MAX_TX_PACKET_TO_OBU)
            return data
        except socket.error as e:
            self.logger.error(f"{e}")
            return None
        
    def get_base(self):
        buf = (c_char * MAX_TX_PACKET_TO_OBU)()
        memset(addressof(buf), 0, sizeof(buf))
        hdr = cast(buf, POINTER(V2x_App_Hdr))
        hdr.contents.magic = V2V_INF_EXT_MAGIC
        
        return buf, hdr

    def set_tx(self):
        self.tx_buf, self.hdr = self.get_base()
        self.tx_msg = cast(addressof(self.hdr.contents) + V2x_App_Hdr.data.offset, POINTER(V2x_App_TxMsg))
        if self.tx_buf == None or self.hdr == None or self.tx_msg == None:
            self.logger.error("V2x_APP_Hdr memory setting error")
            return -1
        else:  
            self.logger.debug("Tx Send Set")
            return 1
    
    def set_rx(self):
        self.rx_buf = (c_char * MAX_TX_PACKET_TO_OBU)()
        self.rx_cnt = 0
        self.tx_cnt_from_rx = 0
        self.rx_rate = 0
        self.rx_latitude = 0
        self.rx_longtude = 0
        
        if self.rx_buf == None:
            self.logger.error("Receive memory setting error")
            return -1
        else:  
            self.logger.debug("Rx Send Set")
            return 1

    def get_p_overall(self, cnt):
        if self.chip == 'out':
            p_overall = cast(addressof(self.tx_msg.contents)+V2x_App_TxMsg.data.offset, POINTER(TLVC_Overall_V2))
        else:
            p_overall = cast(addressof(self.tx_msg.contents)+V2x_App_TxMsg.data.offset, POINTER(TLVC_Overall))
        p_overall.contents.type = socket.htonl(EM_PT_OVERALL)
        p_overall.contents.len = socket.htons(sizeof(TLVC_Overall_V2)-6)
        p_overall.contents.magic=b'EMOP'
        p_overall.contents.version = 1
        p_overall.contents.num_package = cnt
        if self.chip == 'out':
            p_overall.contents.bitwize = 0x77
        crc_data = bytearray(p_overall.contents)
        if self.chip == 'out':
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall_V2)-2))
        else:
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall)-2))
        return p_overall

    def organize_data(self, data_size, sharing_information):

        tx_cnt = socket.ntohl(sharing_information.tx_cnt)
        tx_cnt_from_rx = socket.ntohl(sharing_information.tx_cnt_from_rx)
        vehicle_state = [ sharing_information.state, sharing_information.signal,
                         sharing_information.latitude,sharing_information.longitude,
                         sharing_information.heading, sharing_information.velocity ]
        vehicle_path = [ list(sharing_information.path_x),list(sharing_information.path_y) ]
        return vehicle_state, vehicle_path


    def add_ext_status_data(self, p_overall, package_len):
        if self.chip == 'out':
            p_status = cast(addressof(p_overall.contents)+sizeof(TLVC_Overall_V2)+package_len, POINTER(TLVC_STATUS_CommUnit))
        else:
            p_status = cast(addressof(p_overall.contents)+sizeof(TLVC_Overall)+package_len, POINTER(TLVC_STATUS_CommUnit))
        p_overall.contents.num_package += 1
        package_len += sizeof(TLVC_STATUS_CommUnit)
        p_overall.contents.len_package = socket.htons(package_len)
        crc_data = bytearray(addressof(p_overall.contents))
        if self.chip == 'out':
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall_V2)-2))
        else:
            p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall)-2))
        p_status.contents.type = socket.htonl(EM_PT_STATUS)
        p_status.contents.len = socket.htons(sizeof(TLVC_STATUS_CommUnit)-6)
        p_status.contents.dev_type = eV2x_App_Ext_Status_DevType.eStatusDevType_Obu
        p_status.contents.tx_rx = 0
        p_status.contents.dev_id = socket.htonl(1)
        p_status.contents.hw_ver = socket.htons(2)
        p_status.contents.sw_ver = socket.htons(3)
        p_status.contents.timestamp = self.htobe64(self.get_keti_time())
        crc_data = bytearray(addressof(p_status.contents))
        p_status.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_STATUS_CommUnit)-2))


    def get_keti_time(self):
        now = datetime.now() + timedelta(hours=9)

        # 필요한 포맷으로 시간 조합
        keti_time = (now.year * 1000000000000000 +
                    now.month * 10000000000000 +
                    now.day * 100000000000 +
                    now.hour * 1000000000 +
                    now.minute * 10000000 +
                    now.second * 100000 +
                    now.microsecond // 10) 
        return keti_time

    def htobe64(self, value):
        packed_value = struct.pack('>Q', value)
        return struct.unpack('>Q', packed_value)[0]

    def print_hexa(self, data):
        self.logger.info("Binary data in hexadecimal: ", end="")
        for byte in data:  # buf의 각 바이트에 대해 반복
            print(f"{byte:02X} ", end="")  # 각 바이트를 16진수 형식으로 출력
        print()  # 줄바꿈
    
    def get_log_datum(self, vehicle_state, vehicle_path):
        if len(vehicle_state) > 0:
            state = f"Shared Message\nstate:{vehicle_state[0]} lat:{vehicle_state[2]} lng:{vehicle_state[3]} h:{vehicle_state[4]} v:{vehicle_state[5]}\n"
        else:
            state = "No message to Send\n"
        if vehicle_path != [] and len(vehicle_path[0]) > 1:
            path = f"path: x={vehicle_path[0][0]} y={vehicle_path[0][1]} ~ x={vehicle_path[-1][0]} y={vehicle_path[-1][1]}\n"
        else:
            path = f"Threre is no path\n"
        return state+path

    def get_performance_log(self):
        performance_str = "Communication Performance \n" + " ".join([f"{key}: {value}" for key, value in self.communication_performance.items()])
        return performance_str+"\n\n"