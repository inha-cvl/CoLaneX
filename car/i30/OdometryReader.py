import can
import cantools

class OdometryReader:
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitreate=500000)
        self.db = cantools.database.loadfile('can.dbc')
        self.CAN_data = 0
        self.data = 0
        self.RL_vel = 0
        self.RR_vel = 0
        self.velocity = 0

    def read_odom(self):
        self.CAN_data = self.bus.recv(0)

        if  (self.CAN_data != None) and (self.CAN_data.arbitration_id == 902):
            self.data = self.db.decode_message(self.CAN_data.arbitration_id, self.CAN_data.data)
            self.RL_vel = self.data['WHL_SPD_RL']
            self.RR_vel = self.data['WHL_SPD_RR']
            self.velocity = (self.RL_vel + self.RR_vel)/2

        if self.CAN_data == None:
            self.RL_vel = 0
            self.RR_vel = 0
            self.velocity = 0
    
    def get_vel(self) -> float:
        return self.velocity