import sys
import threading
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

import base_station_gui2.tabbed_window
from rclpy.executors import SingleThreadedExecutor

from base_station_interfaces.srv import BeaconId
from base_station_interfaces.msg import Connections, Status

class GuiNode(Node):
    """
    ROS 2 node that connects the GUI to ROS topics and services.
    Handles publishing, subscribing, and service clients for the GUI.
    """
    def __init__(self, window):
        super().__init__('gui_node')
        # Publisher for sending String messages to the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.kill_subscription = self.create_subscription(
            Bool,
            'confirm_e_kill',
            window.recieve_kill_confirmation_message,  # Calls the GUI's recieve_kill_confirmation_message method
            10)        
            
        self.surface_subscription = self.create_subscription(
            Bool,
            'confirm_e_surface',
            window.recieve_surface_confirmation_message,  # Calls the GUI's recieve_surface_confirmation_message method
            10)

        # Subscription for receiving Connections messages from the 'connections' topic
        self.conn_subscription = self.create_subscription(
            Connections,
            'connections',
            window.recieve_connections,  # Calls the GUI's recieve_connections method
            10)   

        # Subscription for receiving Status messages from the 'status' topic
        self.stat_subscription = self.create_subscription( 
            Status,
            'status',
            window.receive_status,  # Calls the GUI's receive_status method
            10) 
            
        # Service clients for emergency kill and surface services
        self.cli = self.create_client(BeaconId, 'e_kill_service')
        self.cli2 = self.create_client(BeaconId, 'e_surface_service')

    def listener_callback(self, msg):
        """
        Example callback for logging received String messages.
        """
        self.get_logger().info('I heard: "%s"' % msg.data)

    def publish_text(self, text):
        """
        Publishes a String message to the 'topic' topic and logs it.
        """
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing from GUI: "{text}"')

def ros_spin_thread(executor):
    """
    Spins the ROS 2 executor in a background thread.
    """
    executor.spin()

def main():
    """
    Main entry point for the GUI application.
    Initializes ROS 2, starts the Qt application, and spins ROS in a background thread.
    """
    rclpy.init()

    # Create the Qt application and main window
    app, window = base_station_gui2.tabbed_window.OpenWindow(None, borders=False)

    # Create the ROS 2 node and assign it to the GUI window
    gui_node = GuiNode(window)
    window.ros_node = gui_node  # if you need to access the node from the GUI

    # Create a single-threaded executor and add the node
    executor = SingleThreadedExecutor()
    executor.add_node(gui_node)

    # Spin ROS 2 in a background thread so the Qt event loop can run
    ros_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()

    # Ensure Ctrl+C interrupts the Qt event loop
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Start a dummy QTimer to keep the Qt event loop alive
    def start_timer():
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)

    QTimer.singleShot(0, start_timer)

    try:
        # Start the Qt event loop
        exit_code = app.exec()
    finally:
        # Clean up ROS 2 resources on exit
        gui_node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

def SeeAllIcons():
    """
    Utility function to display all available QStyle.StandardPixmap icons in a grid.
    Useful for GUI development and icon selection.
    """
    import sys
    from PyQt6.QtWidgets import (QApplication, QGridLayout, QPushButton, QStyle, QWidget)
    class Window(QWidget):
        def __init__(self):
            super().__init__()

            # Get all standard pixmap icon names
            icons = sorted([attr for attr in dir(QStyle.StandardPixmap) if attr.startswith("SP_")])
            layout = QGridLayout()

            # Create a button for each icon, displaying the icon and its name
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