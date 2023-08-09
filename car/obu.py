import paramiko
import re
import time
import rospy
from geometry_msgs.msg import Pose
import sys
import signal


def signal_handler(sig, frame):
    sys.exit(0)

def get_latitude_longitude(vehicle_type):
    # SSH 접속 정보 설정
    host = '192.168.1.12'
    
    port = 45345
    username = 'sirius'
    password = 'alphacma'
    # SSH 클라이언트 생성
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    rospy.init_node(f'{vehicle_type}_obu', anonymous=False)
    rate = rospy.Rate(10)
    pub_pose = rospy.Publisher(f'/car/{vehicle_type}_pose', Pose, queue_size=1)
    pose = Pose()

    try:
        # SSH 접속
        client.connect(hostname=host, port=port, username=username, password=password)
        chan = client.invoke_shell()
        chan.send('gpspipe -w -n 0\n')

        while not rospy.is_shutdown():
            if chan.recv_ready():
                data = chan.recv(9600).decode('utf-8')
                matches = re.findall(r'"(lat|lon|speed|track)":(-?\d+\.\d+|[\d.]+)', data)

                if len(matches) == 4:
                    latitude, longitude, speed, heading = [float(match[1]) for match in matches]
                    pose.position.x, pose.position.y, pose.position.z, pose.orientation.x = latitude, longitude, heading, speed
                    pub_pose.publish(pose)
                    print(latitude, longitude, speed, heading)
                rate.sleep()

    except paramiko.AuthenticationException:
        print("Authentication failed. Please check your credentials.")
    except paramiko.SSHException as ssh_exc:
        print(f"Error during SSH connection: {ssh_exc}")
    finally:
        # SSH 연결 닫기
        client.close()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 2 or sys.argv[1] not in ['hlv', 'tlv']:
        print("Usage: python your_script_name.py <vehicle_type> (hlv or tlv)")
        return
    vehicle_type = sys.argv[1]
    get_latitude_longitude(vehicle_type)

if __name__ == "__main__":
   main()
