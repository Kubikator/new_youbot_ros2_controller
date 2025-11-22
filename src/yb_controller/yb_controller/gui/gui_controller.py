#!/usr/bin/env python3

import os
import subprocess
import sys
import threading
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from yb_interfaces.msg import LocalizedObjectArray
from yb_controller.gui.qnode import QNode

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('yb_controller')
        ui_file = os.path.join(package_share_dir, 'ui', 'gui_controller.ui')
        
        # Создание примера UI файла, если он не существует
        if not os.path.exists(ui_file):
            print(f"UI file {ui_file} does not exist")
            sys.exit()
            
        uic.loadUi(ui_file, self)  # Укажите путь к .ui файлу
        
        # Инициализация ROS2
        self.ros_node = QNode()
        
        # Настройка элементов управления
        self.b_research.clicked.connect(self.start_exploration)
        self.b_grasping.clicked.connect(self.capture_object)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setScaledContents(True)  # Включаем автомасштабирование
        self.label_1.setAlignment(Qt.AlignCenter)
        self.label_1.setScaledContents(True)  # Включаем автомасштабирование
        
        # Таймер для обновления списка объектов
        from PyQt5.QtCore import QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_objects)
        self.timer.start(1000)  # Обновление каждую секунду
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(50)
        
        # Отдельный поток для ROS2
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

    def start_exploration(self):
        """Обработчик кнопки начала изучения карты"""
        self.ros_node.start_exploration()

    def capture_object(self):
        """Обработчик кнопки захвата объекта"""
        selected = self.cb_objects.currentText()
        if selected:
            self.ros_node.capture_object(selected)

    def update_objects(self):
        """Обновление списка объектов в комбобоксе"""
        current = self.cb_objects.currentText()
        self.cb_objects.clear()
        self.cb_objects.addItems(self.ros_node.detected_objects)

        if current in self.ros_node.detected_objects:
            self.cb_objects.setCurrentText(current)

    def update_gui(self):
        pixmap = QPixmap.fromImage(self.ros_node.curr_image)
        self.label.setPixmap(pixmap)

        pixmap_1 = QPixmap.fromImage(self.ros_node.curr_image_1)
        self.label_1.setPixmap(pixmap_1)

        self.b_grasping.setEnabled(self.ros_node.explore_success)

    def ros_spin(self):
        """Цикл обработки ROS2 сообщений в отдельном потоке"""
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def closeEvent(self, event):
        """Корректное завершение при закрытии окна"""
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    # Инициализация ROS2
    rclpy.init()
    
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        # Корректное завершение ROS2
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()