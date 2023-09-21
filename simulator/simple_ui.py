import sys
import math
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,QWidget, QTabWidget
import rospy
from std_msgs.msg import Float32MultiArray, Int8
from PyQt5.QtCore import Qt, QCoreApplication, QTimer
from rviz import bindings as rviz

app = None

class RvizWidget(rviz.VisualizationFrame):
    def __init__(self, parent=None):
        super(RvizWidget, self).__init__(parent)
        self.setSplashPath('')
        self.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "./rviz/field.rviz")
        self.load(config)
        self.setFixedHeight(500)

class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        rospy.init_node('simple_ui', anonymous=True)


        self.hlv_message = ['대기', '왼쪽으로 차선 변경', '오른쪽으로 차선 변경', '  TLV가 차선 변경 수락', 'TLV가 차선 변경 거절', '종료']
        self.tlv_message = ['대기', '오른쪽 차 합류 요청', '왼쪽 차 합류 요청', '  HLV가 합류 시 안전함', 'HLV가 합류 시 위험함', '종료']

        self.mode_pub = rospy.Publisher("/mode", Int8, queue_size=10)
        self.hlv_signal_pub = rospy.Publisher("/hlv_signal", Int8, queue_size=10)
        self.tlv_signal_pub = rospy.Publisher("/tlv_signal", Int8, queue_size=10)

        self.initUI()

        rospy.Subscriber('/hlv_system', Float32MultiArray, self.hlv_system_cb)
        rospy.Subscriber('/tlv_system', Float32MultiArray, self.tlv_system_cb)
        
        self.hlv_sig = 0
        self.hlv_in = False
        self.hlv_timer = QTimer(self)
        self.hlv_timer.timeout.connect(self.pub_hlv_signal)
        self.tlv_sig = 0
        self.tlv_in = False
        self.tlv_timer = QTimer(self)
        self.tlv_timer.timeout.connect(self.pub_tlv_signal)

    def hlv_system_cb(self, msg):
        self.label_tab1.setText(self.hlv_message[int(msg.data[0])])
        self.side1.setText(f"{math.ceil(msg.data[3])}ms")
        self.side2.setText(f"{round(msg.data[4], 3)}mbps")
        self.side3.setText(f"{int(msg.data[5])}%")
        self.side4.setText(f"{round(msg.data[6], 2)}m")

    def tlv_system_cb(self, msg):
        self.label_tab2.setText(self.tlv_message[int(msg.data[0])])

    def pub_mode(self, mode):
        self.mode_pub.publish(Int8(mode))

    def click_hlv(self, v):
        self.hlv_sig = v
        if not self.hlv_in:
            self.hlv_in = True
            self.hlv_timer.start(500)
            QTimer.singleShot(2000, self.stop_hlv)
        else:
            self.stop_hlv
    
    def click_tlv(self, v):
        self.tlv_sig = v
        if not self.tlv_in:
            self.tlv_in = True
            self.tlv_timer.start(500)
            QTimer.singleShot(2000, self.stop_tlv)
        else:
            self.stop_tlv
    
    def stop_hlv(self):
        self.hlv_in = False
        self.hlv_timer.stop()
        self.pub_hlv_signal()

    def stop_tlv(self):
        self.tlv_in = False
        self.tlv_timer.stop()
        self.pub_tlv_signal()
    
    def pub_hlv_signal(self):
        if self.hlv_in:
            self.hlv_signal_pub.publish(Int8(self.hlv_sig))
        else:
            self.hlv_signal_pub.publish(Int8(0))
        
    def pub_tlv_signal(self):
        if self.tlv_in:
            self.tlv_signal_pub.publish(Int8(self.tlv_sig))
        else:
            self.tlv_signal_pub.publish(Int8(0))

    def initUI(self):
        self.setGeometry(100, 100, 1280,800)
        self.setWindowTitle('CoLaneX')
        
        self.rviz_widget = RvizWidget(self)

        tab_widget = QTabWidget(self)
        tab1 = QWidget()
        tab2 = QWidget()

        button_layout = QHBoxLayout()
        button_enable = QPushButton('On')
        button_enable.setFixedHeight(40)
        button_enable.setStyleSheet('font-size: 20px;')
        button_enable.clicked.connect(lambda:self.pub_mode(1))
        button_disable = QPushButton('Off')
        button_disable.setFixedHeight(40)
        button_disable.setStyleSheet('font-size: 20px;')
        button_disable.clicked.connect(lambda:self.pub_mode(0))
        button_layout.addWidget(button_enable)
        button_layout.addWidget(button_disable)


        button1_tab1 = QPushButton('<', tab1)
        button1_tab1.setFixedHeight(80)
        button1_tab1.setStyleSheet('font-size: 30px;')
        button1_tab1.clicked.connect(lambda:self.click_hlv(1))
        button2_tab1 = QPushButton('>', tab1)
        button2_tab1.setFixedHeight(80)
        button2_tab1.setStyleSheet('font-size: 30px;')
        button2_tab1.clicked.connect(lambda:self.click_hlv(2))

        button1_tab2 = QPushButton('O', tab2)
        button1_tab2.setFixedHeight(80)
        button1_tab2.setStyleSheet('font-size: 30px;')
        button1_tab2.clicked.connect(lambda:self.click_tlv(1))
        button2_tab2 = QPushButton('X', tab2)
        button2_tab2.setFixedHeight(80)
        button2_tab2.setStyleSheet('font-size: 30px;')
        button2_tab2.clicked.connect(lambda:self.click_tlv(2))

        self.label_tab1 = QLabel("대기", tab1)
        self.label_tab1.setAlignment(Qt.AlignCenter)  # Center-align the text
        self.label_tab1.setStyleSheet("font-size: 18px;")  # Increase the font size

        self.label_tab2 = QLabel("대기", tab2)
        self.label_tab2.setAlignment(Qt.AlignCenter)  # Center-align the text
        self.label_tab2.setStyleSheet("font-size: 18px;")  # Increase the font size


        button_layout_tab1 = QHBoxLayout()  # Use QHBoxLayout for horizontal layout
        button_layout_tab1.addWidget(button1_tab1)
        button_layout_tab1.addWidget(button2_tab1)

        layout_tab1 = QVBoxLayout()
        layout_tab1.addLayout(button_layout_tab1)
        layout_tab1.addWidget(self.label_tab1)  

        button_layout_tab2 = QHBoxLayout()  # Use QHBoxLayout for horizontal layout
        button_layout_tab2.addWidget(button1_tab2)
        button_layout_tab2.addWidget(button2_tab2)

        layout_tab2 = QVBoxLayout()
        layout_tab2.addLayout(button_layout_tab2)
        layout_tab2.addWidget(self.label_tab2)  

        tab1.setLayout(layout_tab1)
        tab2.setLayout(layout_tab2)

        tab_widget.addTab(tab1, "HLV")
        tab_widget.addTab(tab2, "TLV")

        
        side0 = QLabel('   Communication performance   ',self)
        side0.setFixedHeight(80)
        side0.setStyleSheet('font-size:16px; font-weight:bold')
        side1 = QLabel("RTT", self)
        side1.setAlignment(Qt.AlignRight)
        side1.setStyleSheet('font-weight:bold')
        self.side1 = QLabel("0ms", self)
        self.side1.setAlignment(Qt.AlignRight)
        side2 = QLabel("Transmission Speed", self)
        side2.setAlignment(Qt.AlignRight)
        side2.setStyleSheet('font-weight:bold')
        self.side2 = QLabel("0mbps", self)
        self.side2.setAlignment(Qt.AlignRight)
        side3 = QLabel("Packet Success", self)
        side3.setAlignment(Qt.AlignRight)
        side3.setStyleSheet('font-weight:bold')
        self.side3 = QLabel("0%", self)
        self.side3.setAlignment(Qt.AlignRight)
        side4 = QLabel("V2V Distance", self)
        side4.setAlignment(Qt.AlignRight)
        side4.setStyleSheet('font-weight:bold')
        self.side4 = QLabel("0m", self)
        self.side4.setAlignment(Qt.AlignRight)

        line1 = QLabel('',self)
        line1.setStyleSheet('background-color:black')
        line1.setFixedHeight(1)
        line2 = QLabel('',self)
        line2.setStyleSheet('background-color:black')
        line2.setFixedHeight(1)
        line3 = QLabel('',self)
        line3.setStyleSheet('background-color:black')
        line3.setFixedHeight(1)
        line4 = QLabel('',self)
        line4.setStyleSheet('background-color:black')
        line4.setFixedHeight(1)
        line5 = QLabel('',self)
        line5.setStyleSheet('background-color:black')
        line5.setFixedHeight(1)

        
        side_layout = QVBoxLayout()
        side_layout.addWidget(side0)
        side_layout.addWidget(line1)
        side_layout.addWidget(side1)
        side_layout.addWidget(self.side1)
        side_layout.addWidget(line2)
        side_layout.addWidget(side2)
        side_layout.addWidget(self.side2)
        side_layout.addWidget(line3)
        side_layout.addWidget(side3)
        side_layout.addWidget(self.side3)
        side_layout.addWidget(line4)
        side_layout.addWidget(side4)
        side_layout.addWidget(self.side4)
        side_layout.addWidget(line5)

        central_widget = QWidget(self)
        central_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.rviz_widget)
        left_layout.addLayout(button_layout)
        top_layout.addLayout(left_layout)
        top_layout.addLayout(side_layout)
        central_layout.addLayout(top_layout)
        central_layout.addWidget(tab_widget)
        central_widget.setLayout(central_layout)
        self.setCentralWidget(central_widget)

def main():
    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app.exec_()
    

if __name__ == '__main__':
    main()
