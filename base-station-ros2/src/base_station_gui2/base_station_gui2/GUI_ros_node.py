import sys
import threading
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


import time
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import FluidPressure, BatteryState

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

import base_station_gui2.tabbed_window
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Path #used to publish the path
from sensor_msgs.msg import NavSatFix #used to publish the origin
from geometry_msgs.msg import PoseStamped

from base_station_interfaces.srv import BeaconId, ModemControl
from base_station_interfaces.msg import Connections, Status, ConsoleLog
from frost_interfaces.msg import SystemStatus, SystemControl, UCommand

class GuiNode(Node):
    """
    ROS 2 node that connects the GUI to ROS topics and services.
    Handles publishing, subscribing, and service clients for the GUI.
    """
    def __init__(self, window, selected_cougs):
        super().__init__('gui_node')
        # Set reliable and transient local QoS profile
        qos_reliable_profile = QoSProfile(depth=5)
        qos_reliable_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        for coug_number in selected_cougs:
            #dynamic subscriptions for the safety status messages
            sub = self.create_subscription(
                SystemStatus,
                f'coug{coug_number}/safety_status',
                lambda msg, n=coug_number: window.recieve_safety_status_message(n, msg),
                10
            )
            setattr(self, f'safety_status_subscription{coug_number}', sub)

            #dynamic subscriptions for the smoothed_output messages
            sub = self.create_subscription(
                Odometry,
                f'coug{coug_number}/smoothed_output',
                lambda msg, n=coug_number: window.recieve_smoothed_output_message(n, msg),
                10
            )
            setattr(self, f'smoothed_ouput_subscription{coug_number}', sub)

            #dynamic subscriptions for the depth_data messages
            sub = self.create_subscription(
                PoseWithCovarianceStamped,
                f'coug{coug_number}/depth_data',
                lambda msg, n=coug_number: window.recieve_depth_data_message(n, msg),
                10
            )
            setattr(self, f'depth_data_subscription{coug_number}', sub)            
            
            #dynamic subscriptions for the pressure data topic
            sub = self.create_subscription(
                FluidPressure,
                f'coug{coug_number}/pressure/data',
                lambda msg, n=coug_number: window.recieve_pressure_data_message(n, msg),
                10
            )
            setattr(self, f'pressure_data_subscription{coug_number}', sub)

            #dynamic subscriptions for the battery/data messages
            sub = self.create_subscription(
                BatteryState,
                f'coug{coug_number}/battery/data',
                lambda msg, n=coug_number: window.recieve_battery_data_message(n, msg),
                10
            )
            setattr(self, f'battery_data_subscription{coug_number}', sub)

            #dynamic publishers for the battery/data messages
            pub = self.create_publisher(
                SystemControl,
                f'/coug{coug_number}/system/status',
                qos_reliable_profile
            )
            setattr(self, f'coug{coug_number}_publisher_', pub)

            #dynamic publishers for the map viz paths
            pub = self.create_publisher(
                Path,
                f'/coug{coug_number}/map_viz_path',
                qos_reliable_profile
            )
            setattr(self, f'coug{coug_number}_path_', pub)

            #dynamic publishers for vehicle fins
            pub = self.create_publisher(
                UCommand,
                f'/coug{coug_number}/kinematics/command',
                qos_reliable_profile
            )
            setattr(self, f'coug{coug_number}_fins_', pub)

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

        #subscription for the console log updates, specific to vehicles. 0 means send to all
        self.console_log_sub = self.create_subscription(
            ConsoleLog,
            'console_log',
            window.handle_console_log,
            10
        ) 

        #publisher for the map viz origin 
        self.origin_pub = self.create_publisher(NavSatFix, '/map_viz_origin', qos_reliable_profile)

        self.console_publisher = self.create_publisher(ConsoleLog, 'console_log', 10)

        # Service clients for emergency kill and surface services
        self.cli = self.create_client(BeaconId, 'e_kill_service')
        self.cli2 = self.create_client(BeaconId, 'e_surface_service')
        self.cli3 = self.create_client(ModemControl, 'modem_shut_off_service')

    def publish_console_log(self, msg_text, msg_num):
        msg = ConsoleLog()
        msg.message = msg_text
        msg.vehicle_number = msg_num
        self.console_publisher.publish(msg)

    def publish_origin(self, origin_msg):
    # origin_msg: tuple(float, float)
        msg = NavSatFix()
        msg.latitude = origin_msg[0]
        msg.longitude = origin_msg[1]
        msg.header.frame_id = 'local_xy_origin'
        self.origin_pub.publish(msg)
        self.get_logger().info(f'Publishing from GUI: "{msg}"')

    def publish_path(self, path_msg, vehicle_number):
    # path_msg: list[tuple(float, float)], vehicle_number: int
        msg = Path()
        for point_tuple in path_msg:
            pose_temp = PoseStamped()
            msg.header.frame_id = 'local_xy_origin'
            pose_temp.pose.position.x = point_tuple[0]
            pose_temp.pose.position.y = point_tuple[1]
            msg.poses.append(pose_temp)

        getattr(self, f'coug{vehicle_number}_path_').publish(msg)
        self.get_logger().info(f'Publishing from GUI: "{msg}"')

    def publish_fins(self, fin_degree, vehicle_number):
        msg = UCommand()
        msg.fin = [fin_degree[0], fin_degree[1], fin_degree[2], float(0)]
        getattr(self, f"coug{vehicle_number}_fins_").publish(msg)

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

    # Create the Qt application and main window (window will be set later)
    app, result, selected_cougs = base_station_gui2.tabbed_window.OpenWindow(None, borders=False)

    def after_window_ready():
        window = result.get('window')
        if window is None:
            # Try again shortly
            QTimer.singleShot(50, after_window_ready)
            return

        # Create the ROS 2 node and assign it to the GUI window
        gui_node = GuiNode(window, selected_cougs)
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

    # Start polling for the window to be ready
    QTimer.singleShot(0, after_window_ready)

    try:
        # Start the Qt event loop
        exit_code = app.exec()
    finally:
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