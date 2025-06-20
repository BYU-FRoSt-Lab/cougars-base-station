# from base_station_interfaces.srv import BeaconId, ModemControl
# import random 
# import rclpy
# from rclpy.node import Node


# class EKillService(Node):

#     def __init__(self):
#         super().__init__('e_kill_service_node')
#         self.srv = self.create_service(BeaconId, 'e_kill_service', self.e_kill_callback)
#         self.srv_surface = self.create_service(BeaconId, 'e_surface_service', self.e_surface_callback)
#         self.srv_modem = self.create_service(ModemControl, 'modem_control_service', self.modem_control_callback)

#     def e_kill_callback(self, request, response):
#         # Log the request (customize as needed)
#         self.get_logger().info(f"Received e_kill_service request: {request}")
#         response.success = random.choice([True, False])
#         return response

#     def e_surface_callback(self, request, response):
#         # Log the request (customize as needed)
#         response.success = random.choice([True, False])
#         self.get_logger().info(f"Received e_surface_service request: {request}")
#         return response    
    
#     def modem_control_callback(self, request, response):
#         # Log the request (customize as needed)
#         response.success = random.choice([True, False])
#         self.get_logger().info(f"Received modem_control_service request: {request}")
#         return response
 
# def main():
#     rclpy.init()
#     node = EKillService()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QAction, QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QLabel,
    QMainWindow,
    QStatusBar,
    QToolBar,
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("My App")

        label = QLabel("Hello!")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.setCentralWidget(label)

        toolbar = QToolBar("My main toolbar")
        toolbar.setIconSize(QSize(16, 16))
        self.addToolBar(toolbar)

        button_action = QAction(QIcon("bug.png"), "&Your button", self)
        button_action.setStatusTip("This is your button")
        button_action.triggered.connect(self.toolbar_button_clicked)
        button_action.setCheckable(True)
        toolbar.addAction(button_action)

        toolbar.addSeparator()

        button_action2 = QAction(QIcon("bug.png"), "Your &button2", self)
        button_action2.setStatusTip("This is your button2")
        button_action2.triggered.connect(self.toolbar_button_clicked)
        button_action2.setCheckable(True)
        toolbar.addAction(button_action2)

        toolbar.addWidget(QLabel("Hello"))
        toolbar.addWidget(QCheckBox())

        self.setStatusBar(QStatusBar(self))

        menu = self.menuBar()

        file_menu = menu.addMenu("&File")
        file_menu.addAction(button_action)
        file_menu.addSeparator()

        file_submenu = file_menu.addMenu("Submenu")
        file_submenu.addAction(button_action2)

    def toolbar_button_clicked(self, s):
        print("click", s)