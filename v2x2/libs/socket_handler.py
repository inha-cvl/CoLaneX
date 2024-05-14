
import socket
from ctypes import *
from datetime import datetime, timedelta
import struct

from .v2x_interface import *
from .crc16 import calc_crc16

DEST_PORT = 47347
V2V_PSID = EM_V2V_MSG
IP = '192.168.1.11'

class SocketHandler:
    def __init__(self, type):
        self.type = type
        self.interface_list = [b'', b'enp4s0', b'enx00e04c6a3d90']
        pass

    def connect(self):
        try:
            self.fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.type > 0:
                interface_name = self.interface_list[self.type]
                try:
                    self.fd.setsockopt(socket.SOL_SOCKET, 25, interface_name)
                except PermissionError:
                    print("[Socket Handler] Permission denied for SO_BINDTODEVICE")
                    return -1
                
            servaddr = socket.getaddrinfo(IP, DEST_PORT, socket.AF_INET, socket.SOCK_STREAM)
            self.fd.connect(servaddr[0][4])
            print("[Socket Handler] Socket Opend")
            return 1
        except socket.error as e:
            print(f"[Socket Handler] {e}")
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

        return self.send(data)
    
    def tx(self, cnt, state, paths):
        p_overall = self.get_p_overall(cnt)
        
        # size = 4
        # p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall), POINTER(V2x_App_Ext_TLVC))
        # p_dummy.contents.type = socket.htonl(EM_PT_RAW_DATA)
        # p_dummy.contents.len = socket.htons(size + 2)
        # p_seq = cast(addressof(p_dummy.contents)+6, POINTER(c_uint32))
        # p_seq.contents.value = socket.htonl(cnt+1) #sequence data input 

        size = sizeof(V2x_App_SI_TLVC)
        p_dummy = cast(addressof(p_overall.contents) + sizeof(TLVC_Overall), POINTER(V2x_App_SI_TLVC))
        p_dummy.contents.type = socket.htonl(EM_PT_RAW_DATA)
        p_dummy.contents.len = socket.htons(size + 2)

        p_share_info = cast(addressof(p_dummy.contents.data), POINTER(SharingInformation))
        p_share_info.contents.tx_cnt = socket.htonl(cnt)
        p_share_info.contents.rx_cnt = socket.htonl(self.rx_cnt+8)
        p_share_info.contents.state = state[0]
        p_share_info.contents.signal = state[1]
        p_share_info.contents.latitude = state[2]
        p_share_info.contents.longitude = state[3]
        p_share_info.contents.heading = state[4]
        p_share_info.contents.velocity = state[5]

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

        self.hdr.contents.len = socket.ntohs(sizeof(V2x_App_Hdr)+6+sizeof(TLVC_Overall)+socket.ntohs(p_overall.contents.len_package))
        self.hdr.contents.seq = 0
        self.hdr.contents.payload_id = socket.htons(0x10)
        self.tx_msg.contents.psid = socket.htonl(EM_V2V_MSG)
        send_len = socket.ntohs(self.hdr.contents.len) + 6

        crc16 = pointer(c_uint16.from_buffer(self.tx_buf, send_len - 2))
        _crc16 = calc_crc16(self.tx_buf[SIZE_MAGIC_NUMBER_OF_HEADER:], send_len-6)
        crc16.contents.value = socket.htons(_crc16)
        
        data = self.tx_buf[:send_len]
        
        self.print_datum(send_len, cnt, self.rx_cnt, state, paths)

        return self.send(data)

    def rx(self):
        data = self.receive()
        if data == None:
            return None
        if len(data) > SIZE_WSR_DATA:
            self.rx_cnt += 1
            hdr_ofs = V2x_App_Hdr.data.offset
            rx_ofs = V2x_App_RxMsg.data.offset
            ovr_ofs = sizeof(TLVC_Overall)
            tlvc_ofs =  hdr_ofs+rx_ofs+ovr_ofs
            tlvc = V2x_App_SI_TLVC.from_buffer_copy(data,tlvc_ofs)
            sharing_information = tlvc.data
            state, path = self.organize_data(len(data), sharing_information)
            return [state, path]
        else:
            return [0, 0]

    def calc_comm(self):
        pass

    def send(self, data):
        try:
            self.fd.sendall(data)
            print("[Socket Handler] Packet Sent")
            return 1
        except socket.error as e:
            print(f"[Socket Handler] {e}")
            return -1
    
    def receive(self):
        try:
            data = self.fd.recv(sizeof(c_char)*MAX_TX_PACKET_TO_OBU)
            print("[Socket Handler] Packet Received")
            return data
        except socket.error as e:
            print(f"[Socket Handler] {e}")
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
            print("[Socket Handler] V2x_APP_Hdr memory setting error")
            return -1
        else:  
            print("[Socket Handler] Tx Send Set")
            return 1
    
    def set_rx(self):
        self.rx_buf = (c_char * MAX_TX_PACKET_TO_OBU)()
        self.rx_cnt = 0
        if self.rx_buf == None:
            print("[Socket Handler] Receive memory setting error")
            return -1
        else:  
            print("[Socket Handler] Rx Send Set")
            return 1

    def get_p_overall(self, cnt):
        p_overall = cast(addressof(self.tx_msg.contents)+V2x_App_TxMsg.data.offset, POINTER(TLVC_Overall))
        p_overall.contents.type = socket.htonl(EM_PT_OVERALL)
        p_overall.contents.len = socket.htons(sizeof(TLVC_Overall)-6)
        p_overall.contents.magic=b'EMOP'
        p_overall.contents.version = 1
        p_overall.contents.num_package = cnt
        crc_data = bytearray(p_overall.contents)
        p_overall.contents.crc = socket.htons(calc_crc16(crc_data, sizeof(TLVC_Overall)-2))
        return p_overall

    def organize_data(self, data_size, sharing_information):

        tx_cnt = socket.ntohl(sharing_information.tx_cnt)
        rx_cnt = socket.ntohl(sharing_information.rx_cnt)
        vehicle_state = [sharing_information.state, sharing_information.signal,
                         sharing_information.latitude,sharing_information.longitude,
                         sharing_information.heading, sharing_information.velocity ]
        vehicle_path = [ list(sharing_information.path_x),list(sharing_information.path_y) ]
        self.print_datum(data_size, tx_cnt, rx_cnt, vehicle_state, vehicle_path)
        return vehicle_state, vehicle_path



    def add_ext_status_data(self, p_overall, package_len):
        p_status = cast(addressof(p_overall.contents)+sizeof(TLVC_Overall)+package_len, POINTER(TLVC_STATUS_CommUnit))
        p_overall.contents.num_package += 1
        package_len += sizeof(TLVC_STATUS_CommUnit)
        p_overall.contents.len_package = socket.htons(package_len)
        crc_data = bytearray(addressof(p_overall.contents))
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
        print("Binary data in hexadecimal: ", end="")
        for byte in data:  # buf의 각 바이트에 대해 반복
            print(f"{byte:02X} ", end="")  # 각 바이트를 16진수 형식으로 출력
        print()  # 줄바꿈
    
    def print_datum(self, data_size, tx_cnt, rx_cnt, vehicle_state, vehicle_path):
        print(f"Print Data\ndata_size:{data_size} tx_cnt:{tx_cnt} rx_cnt:{rx_cnt}")
        print(f"state:{vehicle_state[0]} signal:{vehicle_state[1]}\nlat:{vehicle_state[2]} lng:{vehicle_state[3]} h:{vehicle_state[4]} v:{vehicle_state[5]}")
        print(f"path: x={vehicle_path[0][0]} y={vehicle_path[1][0]} ~ x={vehicle_path[0][-1]} y={vehicle_path[1][-1]}")