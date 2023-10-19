import rospy
from std_msgs.msg import Float32MultiArray
import datetime 
import os
import sys

class RateDistanceCalculator:
    def __init__(self, vehicle_type):
        self.rate_sum = [0.0] * 51  # 0m부터 10m까지의 거리마다 rate의 합을 저장
        self.count = [0] * 51       # 0m부터 10m까지의 거리마다 데이터 개수를 저장

        rospy.init_node('rate_distance_calculator', anonymous=True)
        rospy.Subscriber(f'/{vehicle_type}_system', Float32MultiArray, self.callback)

    def callback(self, data):
        if len(data.data) == 7:
            distance = data.data[6]
            rate = data.data[5]

            # 거리를 10m 단위로 나누어서 인덱스 계산
            distance_index = min(int(distance / 10), 10)
            
            self.rate_sum[distance_index] += rate
            self.count[distance_index] += 1


    def calculate_averages(self):
        now = datetime.datetime.now()
        mmdd = now.strftime("%m%d")

        num = 1
        while True:
            file_name = f"./log/{mmdd}_{vehicle_type}_{num}.txt"
            if not os.path.exists(file_name):
                break
            num += 1

        with open(file_name, 'a') as file:
            for i in range(51):
                if self.count[i] > 0:
                    average_rate = self.rate_sum[i] / self.count[i]
                    file.write(f"Average rate for {i*10}m to {(i+1)*10}m distance: {average_rate:.2f} cnt= [{int(self.count[i])}]\n")
                    # print()

if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1] not in ['hlv', 'tlv']:
        print("Usage: python your_script_name.py <vehicle_type> (hlv or tlv)")
        
    vehicle_type = sys.argv[1]
    calculator = RateDistanceCalculator(vehicle_type)
    rospy.spin()
    calculator.calculate_averages()
