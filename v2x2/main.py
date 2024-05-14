import sys
from ros_manager import RosManager
from v2v_sharing import V2VSharing

def main():
    if len(sys.argv) != 2:
        type = 0
    else:
        type = int(sys.argv[1]) #1: ethernet, 2: usb ethernet
    v2v_sharing = V2VSharing(type)
    ros_manager = RosManager(v2v_sharing, type)
    if ros_manager.execute() < 0:
        print("System Error")
        sys.exit(0)
    else:
        print("System Over")
        sys.exit(0)

if __name__== '__main__':
    main()