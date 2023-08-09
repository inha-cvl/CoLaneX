import rospy
from std_msgs.msg import Float32MultiArray

class RateDistanceCalculator:
    def __init__(self):
        self.rate_sum = [0.0] * 11  # 0m부터 10m까지의 거리마다 rate의 합을 저장
        self.count = [0] * 11       # 0m부터 10m까지의 거리마다 데이터 개수를 저장

        rospy.init_node('rate_distance_calculator', anonymous=True)
        rospy.Subscriber('/hlv_system', Float32MultiArray, self.callback)

    def callback(self, data):
        if len(data.data) == 7:
            distance = data.data[6]
            rate = data.data[5]

            # 거리를 10m 단위로 나누어서 인덱스 계산
            distance_index = min(int(distance / 10), 10)
            
            if self.count[distance_index] <= 50:
                self.rate_sum[distance_index] += rate
                self.count[distance_index] += 1


    def calculate_averages(self):
        for i in range(11):
            if self.count[i] > 0:
                average_rate = self.rate_sum[i] / self.count[i]
                print(f"Average rate for {i*10}m to {(i+1)*10}m distance: {average_rate:.2f} cnt= [{int(self.count[i]/10)}]")

if __name__ == '__main__':
    calculator = RateDistanceCalculator()
    rospy.spin()
    calculator.calculate_averages()
