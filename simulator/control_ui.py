import sys
import math
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton,  QVBoxLayout, QHBoxLayout,QWidget, QTabWidget, QTableWidget, QTableWidgetItem, QLineEdit
import rospy
from std_msgs.msg import Int8, Float32, Float32MultiArray
from geometry_msgs.msg import Pose, Vector3
from PyQt5.QtCore import QTimer
from rviz import bindings as rviz

app = None

from libs.widgets import *

STEER_RATIO = 14.6
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
        self.speed_graph = SpeedSubscriberWidget(self)
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
        self.pid_pub = rospy.Publisher("/pid_params", Vector3, queue_size=1)
        
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
        for i,v in enumerate(msg.data[:5]):
            item = QTableWidgetItem(str(v))
            self.table_widget1.setItem(0, i, item)
        for i,v in enumerate(msg.data[5:12]):
            item = QTableWidgetItem(str(v))
            self.table_widget2.setItem(0, i, item)
        for i,v in enumerate(msg.data[12:]):
            item = QTableWidgetItem(str(v))
            self.table_widget3.setItem(0, i, item)

    def pub_mode(self, mode):
        self.mode_pub.publish(Int8(mode))

    def updateUI(self):
        self.speedometer_widget.set_speed(self.inform['e_v'], self.inform['t_v'])
        self.speed_graph.set_speed(self.inform['e_v'], self.inform['t_v'])
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

    def onSubmit(self):
        # Retrieve values from the input fields
        kkk = Vector3()
        kp = self.kpInput.text()
        ki = self.kiInput.text()
        kd = self.kdInput.text()
        kkk.x = float(kp)
        kkk.y = float(ki)
        kkk.z = float(kd)

        self.pid_pub.publish(kkk)

    def initUI(self):
        self.setGeometry(1080,0,1080,0)
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
        cluster_layout.addWidget(self.speed_graph)
        tab1.setLayout(cluster_layout)
        
        tab2 = QWidget()
        pid_layout = QHBoxLayout()
        pid_layout.addWidget(self.speed_graph)
        k_layout = QVBoxLayout()
        kpLayout = QHBoxLayout()
        kiLayout = QHBoxLayout()
        kdLayout = QHBoxLayout()
        buttonLayout = QHBoxLayout()

         # Kp Input
        self.kpLabel = QLabel('Kp:', self)
        self.kpInput = QLineEdit(self)
        self.kpInput.setText(str(0.65))
        kpLayout.addWidget(self.kpLabel)
        kpLayout.addWidget(self.kpInput)

        # Ki Input
        self.kiLabel = QLabel('Ki:', self)
        self.kiInput = QLineEdit(self)
        self.kiInput.setText(str(0.0001))
        kiLayout.addWidget(self.kiLabel)
        kiLayout.addWidget(self.kiInput)

        # Kd Input
        self.kdLabel = QLabel('Kd:', self)
        self.kdInput = QLineEdit(self)
        self.kdInput.setText(str(0.001))
        kdLayout.addWidget(self.kdLabel)
        kdLayout.addWidget(self.kdInput)

        # Submit Button
        self.submitButton = QPushButton('Submit', self)
        self.submitButton.clicked.connect(self.onSubmit)
        buttonLayout.addWidget(self.submitButton)


        k_layout.addLayout(kpLayout)
        k_layout.addLayout(kiLayout)
        k_layout.addLayout(kdLayout)
        k_layout.addLayout(buttonLayout)
        pid_layout.addLayout(k_layout)
        tab2.setLayout(pid_layout)

        tab3 = QWidget()
        can_layout = QVBoxLayout()
        self.table_widget1 = QTableWidget(self)
        self.table_widget1.setRowCount(1)
        self.table_widget2 = QTableWidget(self)
        self.table_widget2.setRowCount(1)
        self.table_widget3 = QTableWidget(self)
        self.table_widget3.setRowCount(1)
        table1_header = ["PA-Enable","LON-Enable","Accel-Override","Brake-Override","Steering-Override"]
        table2_header = ["Latitude","Longitude","Heading","Velocity","Accel","Brake","Steer"]
        table3_header = ["PA-Enable","LON-Enable","Accel","Brake","Steer","L-Signal","R-Signal","Alive","Reset"]
        self.table_widget1.setColumnCount(len(table1_header))
        self.table_widget1.setHorizontalHeaderLabels(table1_header)
        self.table_widget2.setColumnCount(len(table2_header))
        self.table_widget2.setHorizontalHeaderLabels(table2_header)
        self.table_widget3.setColumnCount(len(table3_header))
        self.table_widget3.setHorizontalHeaderLabels(table3_header)
        table1_label = QLabel("Enable & Override Status", self)
        can_layout.addWidget(table1_label)
        can_layout.addWidget(self.table_widget1)
        table2_label = QLabel("Ego Pose", self)
        can_layout.addWidget(table2_label)
        can_layout.addWidget(self.table_widget2)
        table3_label = QLabel("Target Controls", self)
        can_layout.addWidget(table3_label)
        can_layout.addWidget(self.table_widget3)
        tab3.setLayout(can_layout)

        tab_widget.addTab(tab1, "Cluster")
        tab_widget.addTab(tab2, "PID Tuning")
        tab_widget.addTab(tab3, "CAN")

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
