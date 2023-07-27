from ros_manager import RosManager
from self_drive import SelfDrive
        

def main():
    self_drive = SelfDrive()
    ros_manager = RosManager(self_drive)
    ros_manager.execute()

if __name__ == '__main__':
    main()