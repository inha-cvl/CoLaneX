import can
import cantools

class OdometryReader:
    def __init__(self,
                 dbc_file_path = 'can.dbc',
                 arbitration_id = 640,
                 wheel_vel_rr = 'Gway_Wheel_Velocity_RR',
                 wheel_vel_rl = 'Gway_Wheel_Velocity_RL',
                 ):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitreate=500000)
        self.db = cantools.database.loadfile(dbc_file_path)
        self.CAN_data = 0
        self.data = 0
        self.arbitration_id = arbitration_id
        self.wheel_vel_rr = wheel_vel_rr
        self.wheel_vel_rl = wheel_vel_rl
        self.RL_vel = 0
        self.RR_vel = 0
        self.velocity = 0

    def read_odom(self):
        self.CAN_data = self.bus.recv(0)

        if  (self.CAN_data != None) and (self.CAN_data.arbitration_id == self.arbitration_id):
            self.data = self.db.decode_message(self.CAN_data.arbitration_id, self.CAN_data.data)
            self.RL_vel = self.data[self.wheel_vel_rl]
            self.RR_vel = self.data[self.wheel_vel_rr]
            self.velocity = (self.RL_vel + self.RR_vel)/2

        if self.CAN_data == None:
            self.RL_vel = 0
            self.RR_vel = 0
            self.velocity = 0
    
    def get_vel(self) -> float:
        return self.velocity