from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QFrame
from PyQt5.QtGui import QPainter, QPen, QColor, QTransform, QFont, QBrush, QPixmap, QLinearGradient, QGradient
from PyQt5.QtCore import Qt, QPointF
from rviz import bindings as rviz
import math


class RvizWidget(rviz.VisualizationFrame):
    def __init__(self, parent=None):
        super(RvizWidget, self).__init__(parent)
        self.setSplashPath('')
        self.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "./rviz/selfdrive.rviz")
        self.load(config)
        self.setFixedHeight(400)
        self.setMenuBar(None)
        self.setStatusBar(None)

class SpeedometerWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.speed = 0.0
        self.target_speed = 0.0
        self.setFixedHeight(300)

    def set_speed(self, e,t):
        self.speed = e
        self.target_speed = t
        self.update()
    

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        major_ticks = list(range(210, -31, -40))  
        minor_ticks = list(range(210, -31, -10)) 

        for angle in major_ticks:
            self.draw_tick(painter, angle, major=True)
            self.draw_label(painter, angle) 

        for angle in minor_ticks:
            if angle not in major_ticks:
                self.draw_tick(painter, angle, major=False)

        self.draw_needle(painter, self.target_speed, Qt.red)
        self.draw_needle(painter, self.speed, Qt.black)

        painter.setTransform(QTransform())  
        painter.setFont(QFont('Arial', 20))
        painter.drawText(self.rect().center() +QPointF(-50,50), f"{self.speed}\nkm/h")
    

    def draw_needle(self, painter, speed, color):
        needle_length = min(self.width(), self.height()) / 3.5
        needle_angle = 210 - speed
        if needle_angle <= 0:
            needle_angle = -needle_angle
        needle_angle -= 90
        needle_angle = 360 - needle_angle
        needle_pivot = self.rect().center()
        painter.setPen(QPen(color, 2))
        transform = QTransform().translate(needle_pivot.x(), needle_pivot.y()).rotate(needle_angle).translate(-needle_pivot.x(), -needle_pivot.y())
        painter.setTransform(transform)
        painter.drawLine(needle_pivot, needle_pivot - QPointF(0, needle_length))

    def draw_tick(self, painter, angle, major=True):
        tick_length = 10 if major else 5
        tick_width = 2

        center = self.rect().center()
        radius = min(self.width(), self.height()) / 2.0 - 30  

        start_point = center + QPointF(radius * math.cos(math.radians(angle)), -radius * math.sin(math.radians(angle)))
        end_point = start_point + QPointF(tick_length * math.cos(math.radians(angle)), -tick_length * math.sin(math.radians(angle)))

        painter.setPen(QPen(Qt.black, tick_width))
        painter.drawLine(start_point, end_point)
        

    def draw_label(self, painter, angle):
        if angle-210 <= 0:
            value = -(angle-210)
        else:
            value = angle

        center = self.rect().center()
        radius = min(self.width(), self.height())/2.0 - 20  
        label_radius = radius + 10 

        # Position for the label text
        text_position = center + QPointF(label_radius * math.cos(math.radians(angle)), -label_radius * math.sin(math.radians(angle)))

        painter.setPen(QPen(Qt.black))
        painter.setFont(QFont('Arial', 10))
        painter.drawText(text_position, str(value))

class WheelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.target_yaw = 0.0
        self.setFixedHeight(300)

    def set_yaw(self, e,t):
        self.yaw = e
        self.target_yaw = t
        self.update()


    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        wheel_radius = min(self.width(), self.height())/2-30
        wheel_center = self.rect().center()

        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.drawEllipse(wheel_center, wheel_radius, wheel_radius)

        self.draw_line(painter, wheel_center, self.target_yaw,  Qt.red)
        self.draw_line(painter, wheel_center, self.yaw, Qt.black)

        painter.setFont(QFont('Arial', 20))
        painter.drawText(wheel_center+QPointF(-50, 50), f"{self.yaw:.2f}\ndeg")

    def draw_line(self, painter, center, angle, color):
        if angle < 0:
            dg = 90 + -angle
        else:
            dg = 90 + angle

        line_length = min(self.width(), self.height()) / 3.5
        line_end = center + QPointF(line_length * math.cos(math.radians(-dg)),
                                    line_length * math.sin(math.radians(-dg)))

        painter.setPen(QPen(color, 2))
        painter.drawLine(center, line_end)


class GaugeWidget(QWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.value = 0
        self.gauge_label = QLabel(self)
        self.gauge_label.setAlignment(Qt.AlignCenter)
        self.update_gauge()
        layout = QVBoxLayout()
        layout.addWidget(self.gauge_label)
        label = QLabel(self)
        label.setAlignment(Qt.AlignCenter)
        label.setText(title)
        layout.addWidget(label)
        self.setLayout(layout)
    
    def update_gauge(self):
        gradient = QLinearGradient(0, 0, 0, 250)
        gradient.setColorAt(0, QColor(255, 0, 0, max(30, int(255 * (self.value / 100)))))  # 빨간색 (시작)
        gradient.setColorAt(1, QColor(255, 255, 0, int(255 * (self.value / 100))))  # 노란색 (끝)
        brush = QBrush(gradient)
        pixmap = self.draw_gauge(brush)
        self.gauge_label.setPixmap(pixmap)

    def draw_gauge(self, brush):
        pixmap = self.create_gauge_pixmap(50, 250, brush)
        return pixmap

    def create_gauge_pixmap(self, width, height, brush):
        pixmap = QPixmap(width, height)
        pixmap.fill(Qt.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        gauge_rect = pixmap.rect()
        fill_height = int((self.value / 100) * pixmap.height())
        pos = gauge_rect.height()-fill_height
        fill_rect = gauge_rect.translated(0, pos)
        painter.setBrush(brush)
        painter.drawRect(fill_rect)
        painter.setFont(QFont('Arial', 10))
        painter.drawText(QPointF(10, pos-10), f"{self.value:.2f}")

        painter.end()

        return pixmap

    def set_value(self, value):
        self.value = max(0, min(100, value))
        self.update_gauge()
