import sys
import threading
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import QApplication

import base_station_gui2.tabbed_window
from rclpy.executors import SingleThreadedExecutor

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing from GUI: "{text}"')

class MinimalSubscriber(Node):
    def __init__(self, window):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            # self.listener_callback,
            window.recieve_message,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def ros_spin_thread(executor):
    executor.spin()

def main():
    import signal
    from PyQt5.QtCore import QTimer

    rclpy.init()
    pub_node = MinimalPublisher()

    app, window = base_station_gui2.tabbed_window.OpenWindow(pub_node)
    global main_window
    main_window = window

    sub_node = MinimalSubscriber(main_window)

    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    # Spin the executor in the background
    ros_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()



    # pub_thread = threading.Thread(target=ros_spin_thread, args=(pub_node,), daemon=True)
    # pub_thread.start()

    # sub_thread = threading.Thread(target=ros_spin_thread, args=(sub_node,), daemon=True)
    # sub_thread.start()


    signal.signal(signal.SIGINT, signal.SIG_DFL)

    def start_timer():
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(100)

    # Start timer right after app is fully running
    QTimer.singleShot(0, start_timer)

    try:
        exit_code = app.exec()
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

if __name__ == '__main__':
    main()
