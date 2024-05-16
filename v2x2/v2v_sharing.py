#!/usr/bin/env python
from libs.socket_handler import SocketHandler

class V2VSharing:
    def __init__(self, type):
        self.socket_handler = SocketHandler(type)
        self.type = type
    
    def set_obu(self):
        if self.socket_handler.connect() < 0:
            print("[V2V Sharing] Connection Failed")
            return -1
        if self.socket_handler.register() < 0:
            print("[V2V Sharing] Registratino Failed")
            return -1
        if  self.socket_handler.set_tx() < 0:
            print("[V2V Sharing] Tx Setting Failed")
            return -1
        if  self.socket_handler.set_rx() < 0:
            print("[V2V Sharing] Rx Setting Failed")
            return -1
        return 1


    def do_tx(self, state, paths):
        return self.socket_handler.tx(state, paths)

    def do_rx(self):
        return self.socket_handler.rx()

    def do_calc(self):
        return self.socket_handler.calc_comm()

    def do_calc_rate(self, Hz):
        return self.socket_handler.calc_rate(Hz)

