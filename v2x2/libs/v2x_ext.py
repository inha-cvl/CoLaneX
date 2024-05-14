from ctypes import *

class SharingInformation(Structure):
    _pack_ = 1
    _fields_ = [
        ("tx_cnt", c_uint32),
        ("rx_cnt", c_uint32),
        ("state", c_uint8),
        ("signal", c_uint8),
        ("latitude", c_float),
        ("longitude", c_float),
        ("heading", c_float),
        ("velocity", c_float),
    
        ("path_x", c_float*50),
        ("path_y", c_float*50),
    ]


class V2x_App_Hdr(Structure):
    _pack_ = 1
    _fields_ = [("magic", c_char * 4),
                ("len", c_uint16),
                ("seq", c_uint16),
                ("payload_id", c_uint16),
                ("data", c_uint8*0)]

class V2x_App_WSR_Add_Crc(Structure):
    _pack_ = 1
    _fields_ = [("action", c_uint8),
                ("psid", c_uint32),
                ("crc", c_uint16)]

class V2x_App_Ext_TLVC(Structure):
    _pack_ = 1
    _fields_ = [("type", c_uint32),
                ("len", c_uint16),
                ("data", c_uint8 * 0)]  # Variable-sized data


class V2x_App_SI_TLVC(Structure):
    _pack_ = 1
    _fields_ = [("type", c_uint32),
                ("len", c_uint16),
                ("data", SharingInformation)]  # Variable-sized data
    

class V2x_App_TxMsg(Structure):
    _pack_ = 1
    _fields_ = [("psid", c_uint32),
                ("data", c_uint8 * 0)]  # Variable-sized data, handle appropriately

class V2x_App_RxMsg(Structure):
    _pack_ = 1
    _fields_ = [("psid", c_uint32),
                ("rcpi", c_uint8),
                ("data", c_uint8 * 0)]  # Variable-sized data

class TLVC_Overall(Structure):
    _pack_ = 1  # 메모리 정렬을 1바이트 단위로 설정
    _fields_ = [
        ("type", c_uint32),         # 4 bytes, 네트워크 바이트 오더로 설정해야 함
        ("len", c_uint16),          # 2 bytes, 네트워크 바이트 오더로 설정해야 함
        ("magic", c_char * 4),      # 4 bytes
        ("version", c_uint8),       # 1 byte
        ("num_package", c_uint8),   # 1 byte
        ("len_package", c_uint16),  # 2 bytes, 네트워크 바이트 오더로 설정해야 함
        ("crc", c_uint16)           # 2 bytes, 네트워크 바이트 오더로 설정해야 함
    ]

class TLVC_STATUS_CommUnit(Structure):
    _pack_ = 1  # 모든 필드가 메모리에 밀집되게 패킹
    _fields_ = [
        ("type", c_uint32),       # 4 bytes, 네트워크 바이트 오더
        ("len", c_uint16),        # 2 bytes, 네트워크 바이트 오더
        ("dev_type", c_uint8),    # 1 byte
        ("tx_rx", c_uint8),       # 1 byte
        ("dev_id", c_uint32),     # 4 bytes
        ("hw_ver", c_uint16),     # 2 bytes
        ("sw_ver", c_uint16),     # 2 bytes
        ("timestamp", c_uint64),  # 8 bytes
        ("crc", c_uint16)         # 2 bytes, 네트워크 바이트 오더
    ]
