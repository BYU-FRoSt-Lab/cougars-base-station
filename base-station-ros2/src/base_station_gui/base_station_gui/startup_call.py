#!/usr/bin/env python3
from frost_interfaces.msg import SystemControl  # Change to your actual package name
from base_station_interfaces.msg import ConsoleLog
from std_msgs.msg import Header, Bool
from rclpy.node import Node
global deployment_node

class DeploymentPublisher(Node):
    def __init__(self):
        super().__init__('deployment_publisher')
        self.publisher_ = self.create_publisher(ConsoleLog, 'console_log', 10)

    def publish_log(self, passed_msg):
        self.publisher_.publish(passed_msg)
        self.get_logger().info(f"Published: {passed_msg.message} to Coug#{passed_msg.vehicle_number}")

def publish_console_log(msg_text, msg_num):
    global deployment_node
    msg = ConsoleLog()
    msg.message = msg_text
    msg.vehicle_number = msg_num
    deployment_node.publish_log(msg)

def publish_system_control(node, sel_vehicles, start_config):
    # example of start_config
    # {'start_node': False, 'record_rosbag': False, 'rosbag_prefix': False, 'arm_thruster': True, 'start_dvl': False}
    global deployment_node
    deployment_node = DeploymentPublisher()
    msg = SystemControl()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'system_status_input'

    try:
        for vehicle in sel_vehicles:
            msg.start = Bool(data=start_config["start_node"])
            msg.rosbag_flag = Bool(data=start_config["record_rosbag"])
            msg.rosbag_prefix = start_config["rosbag_prefix"]
            msg.thruster_arm = Bool(data=start_config["arm_thruster"])
            msg.dvl_acoustics = Bool(data=start_config["start_dvl"])

            publisher = getattr(node, f"coug{vehicle}_publisher_", None)
            if publisher is not None:
                publisher.publish(msg)
                node.get_logger().info("Published SystemControl message.")
                node.get_logger().info(
                    f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}"
                )
                publish_console_log(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}", vehicle)  
            else:
                node.get_logger().error(f"No publisher found for coug{vehicle}")

    except Exception as e:
        node.get_logger().error(f"Error getting user input: {e}")