#!/usr/bin/env python3
from cougars_interfaces.msg import SystemControl
from base_station_interfaces.msg import ConsoleLog
from std_msgs.msg import Header, Bool
from rclpy.node import Node
#The passed ros node from tabbed_window.py, created in GUI_ros_node.py
global ros_node

def publish_system_control(node, sel_vehicles, start_config):
    # example of start_config
    # {'start_node': False, 'record_rosbag': False, 'rosbag_prefix': False, 'arm_thruster': True, 'start_dvl': False}
    global ros_node
    ros_node = node
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
                ros_node.publish_console_log(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}, Thruster: {msg.thruster_arm.data}, DVL: {msg.dvl_acoustics.data}", vehicle)  
            else:
                node.get_logger().error(f"No publisher found for coug{vehicle}")

    except Exception as e:
        node.get_logger().error(f"Error getting user input: {e}")