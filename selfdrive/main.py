import sys
from ros_manager import RosManager
from self_drive import SelfDrive

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ['hlv', 'tlv']:
        print("Usage: python your_script_name.py <vehicle_type> (hlv or tlv)")
        return
    vehicle_type = sys.argv[1]
    self_drive = SelfDrive()
    ros_manager = RosManager(self_drive,vehicle_type)
    ros_manager.execute()

if __name__ == '__main__':
    main()