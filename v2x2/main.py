import sys
from ros_manager import RosManager
from v2v_sharing import V2VSharing

def main():
    if len(sys.argv) != 4:
        type = 1
        interface = 0
        chip = 'in'
    else:
        type = int(sys.argv[1]) # hlv:1, tlv:2
        interface = int(sys.argv[2]) #0: local, 1: ethernet, 2: usb ethernet
        chip = str(sys.argv[3]) # in, out
    
    v2v_sharing = V2VSharing(type, interface, chip)
        
    ros_manager = RosManager(v2v_sharing, type)
    if ros_manager.execute() < 0:
        print("System Error")
        sys.exit(0)
    else:
        print("System Over")
        sys.exit(0)

if __name__== '__main__':
    main()