import sys
import math
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton,  QVBoxLayout, QHBoxLayout,QWidget, QTabWidget
import rospy
from std_msgs.msg import Int8, Float32
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
        self.hlv_in = False
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
        self.mode_pub = rospy.Publisher("/mode", Int8, queue_size=1)
        self.hlv_signal_pub = rospy.Publisher("/hlv_signal", Int8, queue_size=10)
        
        # QTimer 인스턴스 생성
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)
        self.inform = {'e_v':0, 't_v':0, 'e_y':0,'t_y':0, 'e_a':0, 't_a':0, 'e_b':0, 't_b':0}

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
            QTimer.singleShot(2000, self.stop_hlv)
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
        self.setGeometry(4000, 100, 1920,1080)
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
        button1_layout.addWidget(enable)
        button1_layout.addWidget(disable)
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

        tab_widget = QTabWidget(self)
        tab1 = QWidget()
        cluster_layout = QHBoxLayout()
        cluster_layout.addWidget(self.speedometer_widget)
        cluster_layout.addWidget(self.wheel_widget)
        cluster_layout.addWidget(self.accel_widget)
        cluster_layout.addWidget(self.brake_widget)

        tab1.setLayout(cluster_layout)
        tab2 = QWidget()
        tab_widget.addTab(tab1, "Cluster")
        tab_widget.addTab(tab2, "-")

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