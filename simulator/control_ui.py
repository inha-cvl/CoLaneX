import sys
import math
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton,  QVBoxLayout, QHBoxLayout,QWidget, QTabWidget
import rospy
from std_msgs.msg import Int8, Float32, Float32MultiArray
from geometry_msgs.msg import Pose, Vector3
from PyQt5.QtCore import QTimer
from rviz import bindings as rviz

app = None

from libs.widgets import *

STEER_RATIO = 13.5
MPS_TO_KPH = 3.6

class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.hlv_sig = 0
        self.hlv_merged = 0
        self.hlv_in = False

        self.inform = {'e_v':0, 't_v':0, 'e_y':0,'t_y':0, 'e_a':0, 't_a':0, 'e_b':0, 't_b':0}

        self.hlv_message = ['대기', '왼쪽으로 차선 변경', '오른쪽으로 차선 변경', '  TLV가 차선 변경 수락', 'TLV가 차선 변경 거절', '종료']

        self.hlv_timer = QTimer(self)
        self.hlv_timer.timeout.connect(self.pub_hlv_signal)

        self.rviz_widget = RvizWidget(self)
        self.speedometer_widget = SpeedometerWidget(self)
        self.wheel_widget = WheelWidget(self)
        self.accel_widget = GaugeWidget("Accel", self)
        self.brake_widget = GaugeWidget("Brake", self)

        self.initUI()

        rospy.Subscriber('/car/hlv_pose',Pose, self.hlv_pose_cb)
        rospy.Subscriber('/selfdrive/hlv_actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/selfdrive/hlv_target_velocity', Float32, self.target_velocity_cb)
        rospy.Subscriber('/hlv_system', Float32MultiArray, self.hlv_system_cb)
        rospy.Subscriber('/car/can', Float32MultiArray, self.can_cb)
        self.mode_pub = rospy.Publisher("/mode", Int8, queue_size=1)
        self.hlv_signal_pub = rospy.Publisher("/hlv_signal", Int8, queue_size=10)
        
        # QTimer 인스턴스 생성
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)
        
    def hlv_system_cb(self, msg):
        self.label_tab1.setText(self.hlv_message[int(msg.data[0])])

    def hlv_pose_cb(self, msg):
        self.inform['e_v'] = int(msg.orientation.x*MPS_TO_KPH)
        self.inform['e_y'] = msg.orientation.y
        self.inform['e_a'] = msg.orientation.z
        self.inform['e_b'] = msg.orientation.w
        
    def actuator_cb(self, msg):
        self.inform['t_y'] = msg.x
        self.inform['t_a'] = msg.y
        self.inform['t_b'] = msg.z

    def target_velocity_cb(self, msg):
        self.inform['t_v'] = int(msg.data*MPS_TO_KPH)
    
    def can_cb(self, msg):
        s1 = "PA-Enable   LON-Enable   Accel-Override   Brake-Override   Steering-Override   Latitude   Longitude   Heading   Velocity   Accel   Brake   Steer   PA-Enable  LON-Enable  Accel  Brake  Steer  L-Signal  R-Signal  Alive  Reset\n"
        s2 = ''
        for m in msg.data:
            s2 = s2+'   '+(str(m))
        s = s1 + s2
        self.can_table.setText(s)

    def pub_mode(self, mode):
        self.mode_pub.publish(Int8(mode))

    def updateUI(self):
        self.speedometer_widget.set_speed(self.inform['e_v'], self.inform['t_v'])
        self.wheel_widget.set_yaw(self.inform['e_y'], self.inform['t_y'])
        self.accel_widget.set_value(self.inform['e_a'])
        self.brake_widget.set_value(self.inform['e_b'])
        self.accel_widget.set_target(self.inform['t_a'])
        self.brake_widget.set_target(self.inform['t_b'])


    def click_hlv(self, v):
        self.hlv_sig = v
        if not self.hlv_in:
            self.hlv_in = True
            self.hlv_timer.start(500)
            QTimer.singleShot(5000, self.stop_hlv)
        else:
            self.stop_hlv
    
    def stop_hlv(self):
        self.hlv_in = False
        self.hlv_timer.stop()
        self.pub_hlv_signal()

    def pub_hlv_signal(self):
        if self.hlv_in:
            self.hlv_signal_pub.publish(Int8(self.hlv_sig))
        else:
            self.hlv_signal_pub.publish(Int8(0))
        
    def initUI(self):
        self.setGeometry(0,0,1920,1080)
        self.setWindowTitle('CoLaneX - Selfdrive')

        button1_layout = QHBoxLayout()
        enable = QPushButton('On')
        enable.setFixedHeight(40)
        enable.setStyleSheet('font-size: 20px;')
        enable.clicked.connect(lambda:self.pub_mode(1))
        disable = QPushButton('Off')
        disable.setFixedHeight(40)
        disable.setStyleSheet('font-size: 20px;')
        disable.clicked.connect(lambda:self.pub_mode(0))
        merge_reset = QPushButton('Merge Reset')
        merge_reset.setFixedHeight(40)
        merge_reset.setStyleSheet('font-size: 20px;')
        merge_reset.clicked.connect(lambda:self.click_hlv(4))
        button1_layout.addWidget(enable)
        button1_layout.addWidget(disable)
        button1_layout.addWidget(merge_reset)
        button2_layout = QHBoxLayout()
        left = QPushButton('<')
        left.setFixedHeight(40)
        left.setStyleSheet('font-size: 40px; font-weight: bold;')
        left.clicked.connect(lambda:self.click_hlv(1))
        straight = QPushButton('^')
        straight.setFixedHeight(40)
        straight.setStyleSheet('font-size: 40px; font-weight: bold;')
        straight.clicked.connect(lambda:self.click_hlv(3))
        right = QPushButton('>')
        right.setFixedHeight(40)
        right.setStyleSheet('font-size: 40px; font-weight: bold;')
        right.clicked.connect(lambda:self.click_hlv(2))
        button2_layout.addWidget(left)
        button2_layout.addWidget(straight)
        button2_layout.addWidget(right)
        button_layout = QVBoxLayout()
        button_layout.addLayout(button1_layout)
        button_layout.addLayout(button2_layout)

        self.label_tab1 = QLabel("대기", self)
        self.label_tab1.setAlignment(Qt.AlignCenter)  # Center-align the text
        self.label_tab1.setStyleSheet("font-size: 30px; font-weight: bold;")  # Increase the font size
        button_layout.addWidget(self.label_tab1)

        tab_widget = QTabWidget(self)
        tab1 = QWidget()
        cluster_layout = QHBoxLayout()
        cluster_layout.addWidget(self.speedometer_widget)
        cluster_layout.addWidget(self.wheel_widget)
        cluster_layout.addWidget(self.accel_widget)
        cluster_layout.addWidget(self.brake_widget)
        tab1.setLayout(cluster_layout)
        
        tab2 = QWidget()
        can_layout = QHBoxLayout()
        self.can_table = QLabel('table', self)
        can_layout.addWidget(self.can_table)
        tab2.setLayout(can_layout)


        tab_widget.addTab(tab1, "Cluster")
        tab_widget.addTab(tab2, "CAN")

        central_widget = QWidget(self)
        central_layout = QVBoxLayout()
        top_layout = QVBoxLayout()
        top_layout.addWidget(self.rviz_widget)
        top_layout.addLayout(button_layout)
        central_layout.addLayout(top_layout)
        central_layout.addWidget(tab_widget)
        central_widget.setLayout(central_layout)
        self.setCentralWidget(central_widget)

def main():
    rospy.init_node('control_ui', anonymous=True)

    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app.exec_()

if __name__ == '__main__':
    main()
