import sys
import threading
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

import base_station_gui2.tabbed_window
from rclpy.executors import SingleThreadedExecutor

from base_station_interfaces.srv import BeaconId
from base_station_interfaces.msg import Connections, Status

class GuiNode(Node):
    def __init__(self, window):
        super().__init__('gui_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            window.recieve_message,
            10)

        self.conn_subscription = self.create_subscription(
            Connections,
            'connections',
            window.recieve_connections,
            10)   

        self.stat_subscription = self.create_subscription(
            Status,
            'status',
            window.receive_status,
            10)
            
        # self.srv = self.create_service(Bool, 'e_kill_service', self.add_two_ints_callback)
        self.cli = self.create_client(BeaconId, 'emergency_kill_service')
        self.cli2 = self.create_client(BeaconId, 'emergency_surface_service')
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing from GUI: "{text}"')

def ros_spin_thread(executor):
    executor.spin()

def main():
    rclpy.init()

    app, window = base_station_gui2.tabbed_window.OpenWindow(None, borders=False)

    # Create GuiNode and assign to GUI
    gui_node = GuiNode(window)
    window.ros_node = gui_node  # if you need to access the node from the GUI

    executor = SingleThreadedExecutor()
    executor.add_node(gui_node)

    # Spin ROS 2 in a background thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    def start_timer():
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)

    QTimer.singleShot(0, start_timer)

    try:
        exit_code = app.exec()
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

def SeeAllIcons():
    import sys
    from PyQt6.QtWidgets import (QApplication, QGridLayout, QPushButton, QStyle, QWidget)
    class Window(QWidget):
        def __init__(self):
            super().__init__()

            icons = sorted([attr for attr in dir(QStyle.StandardPixmap) if attr.startswith("SP_")])
            layout = QGridLayout()

            for n, name in enumerate(icons):
                btn = QPushButton(name)
                pixmapi = getattr(QStyle.StandardPixmap, name)
                icon = self.style().standardIcon(pixmapi)
                btn.setIcon(icon)
                layout.addWidget(btn, int(n/4), int(n%4))
            self.setLayout(layout)
    app = QApplication(sys.argv)
    w = Window()
    w.show()
    app.exec()

if __name__ == '__main__':
    main()