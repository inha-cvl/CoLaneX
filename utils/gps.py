import paramiko
import re
import time
import rospy
from geometry_msgs.msg import Pose

def get_latitude_longitude():
    # SSH 접속 정보 설정
    host = '192.168.1.12'
    port = 45345
    username = 'sirius'
    password = 'alphacma'
    # SSH 클라이언트 생성
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    rospy.init_node('OBU', anonymous=False)
    pub_pose = rospy.Publisher('/car/hlv_pose', Pose, queue_size=1)
    pose = Pose()


    try:
        # SSH 접속
        client.connect(hostname=host, port=port, username=username, password=password)
        chan = client.invoke_shell()
        chan.send('gpspipe -w -n 0\n')

        while True:
            time.sleep(0.5)  # 예: 2초간 대기
            if chan.recv_ready():
                data = chan.recv(9600).decode('utf-8')
                latitude_match = re.search(r'"lat":(-?\d+\.\d+)', data)
                longitude_match = re.search(r'"lon":(-?\d+\.\d+)', data)
                speed_match = re.search(r'"speed":([\d.]+)', data)
                heading_match = re.search(r'"track":([\d.]+)', data)

                if latitude_match and longitude_match and speed_match and heading_match:
                    latitude = float(latitude_match.group(1))
                    longitude = float(longitude_match.group(1))
                    speed = float(speed_match.group(1))
                    heading = float(heading_match.group(1))

                    pose.position.x = latitude
                    pose.position.y = longitude
                    pose.position.z = heading 
                    pose.orientation.x = speed
                    pub_pose.publish(pose)
                    print(latitude, longitude, speed, heading)
                else:
                    print("Unable to extract latitude, longitude, and speed from the data.")

    except paramiko.AuthenticationException:
        print("Authentication failed. Please check your credentials.")
    except paramiko.SSHException as ssh_exc:
        print(f"Error during SSH connection: {ssh_exc}")
    finally:
        # SSH 연결 닫기
        client.close()


if __name__ == "__main__":
    get_latitude_longitude()
