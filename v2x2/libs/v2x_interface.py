from .v2x_ext import *
from ctypes import *

SIZE_V2X_APP_EXT_HEADER = sizeof(V2x_App_Hdr)
SIZE_TX_HEADER = sizeof(V2x_App_TxMsg)
SIZE_RX_HEADER = sizeof(V2x_App_RxMsg)
SIZE_EXT_HEADER = sizeof(V2x_App_Ext_TLVC)
SIZE_WSR_DATA = sizeof(V2x_App_WSR_Add_Crc) + SIZE_V2X_APP_EXT_HEADER

SIZE_MAGIC_NUMBER_OF_HEADER = 4  # uint32_t
SIZE_LENTH_OF_HEADER = 2  # uint16_t
SIZE_CRC16_OF_HEADER = 2  # uint16_t

MAX_PSID_VALUE = 270549119
MAX_DATA_SIZE = 8999  # 최대 데이터 크기

V2V_INF_EXT_MAGIC  = b"5GVX"
EM_V2V_MSG = 58200
EM_PT_OVERALL = 58220
EM_PT_RAW_DATA = 58221
EM_PT_STATUS = 58223

MAX_TX_PACKET_TO_OBU = (MAX_DATA_SIZE + SIZE_V2X_APP_EXT_HEADER + SIZE_TX_HEADER + SIZE_CRC16_OF_HEADER)
MAX_RX_PACKET_BY_OBU = (MAX_DATA_SIZE + SIZE_V2X_APP_EXT_HEADER + SIZE_RX_HEADER + SIZE_CRC16_OF_HEADER)
MAX_RX_LEN_FROM_DEV = (MAX_DATA_SIZE + SIZE_MAGIC_NUMBER_OF_HEADER - SIZE_LENTH_OF_HEADER)



from enum import Enum, IntEnum 

class eNrV2xCmd(Enum):
    CMD_WSM_SERVICE_REQ = 1
    CMD_SEND_DATA = 2
    CMD_SEND_TEST_BSM = 3
    CMD_SEND_TEST_PVD = 4
    CMD_SEND_TEST_EXTENSIBLE_V2V = 5
    CMD_SEND_TEST_EXTENSIBLE_V2I = 6
    CMD_SEND_TEST_EXTENSIBLE_V2V_ADD_SEQUENCE = 7
    CMD_SEND_TEST_EXTENSIBLE_V2I_ADD_SEQUENCE = 8
    CMD_MAX = 9

class eV2x_App_Payload_Id(IntEnum):
    reserved0 = 0x00
    ePayloadId_ACK = 1
    ePayloadId_NAK = 2
    ePayloadId_heartbeat = 3
    ePayloadId_time = 4
    ePayloadId_status = 5
    ePayloadId_repair = 6
    ePayloadId_cmd = 7
    ePayloadId_TxMsg = 0x10
    ePayloadId_RxMsg = 0x11
    ePayloadId_WsmServiceReq = 0x12
    ePayloadId_WsmServiceConfirm = 0x13
    ePayloadId_MAX = ePayloadId_WsmServiceConfirm

class eV2x_App_WSC_Action_Result(IntEnum):
    eWSCActionResult_Fail = 0x00
    eWSCActionResult_Add_OK = 1
    eWSCActionResult_Already_Add = 2
    eWSCActionResult_Del_OK = 3
    eWSCActionResult_Not_Exist_Del = 4

class eV2x_App_Ext_Status_DevType(IntEnum):
    eStatusDevType_ObuModem = 10
    eStatusDevType_Obu = 11
    eStatusDevType_RsuModem = 20
    eStatusDevType_Rsu = 21
    eStatusDevType_RsuControl = 22

class eV2x_App_Ext_Status_TxRx(IntEnum):
    eStatusTxRx_Tx = 0
    eStatusTxRx_Rx = 1