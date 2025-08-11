#!/usr/bin/env python3
from frost_interfaces.msg import SystemControl
from base_station_interfaces.msg import ConsoleLog
from base_station_interfaces.srv import Init
from std_msgs.msg import Header, Bool
import rclpy

def publish_system_control(node, sel_vehicles, start_config, wifi_connection):
    # example of start_config
    # {'start_node': False, 'record_rosbag': False, 'rosbag_prefix': False, 'arm_thruster': True, 'start_dvl': False}
    ros_node = node
    

    try:
        for vehicle in sel_vehicles:
            if wifi_connection[vehicle]:
                msg = SystemControl()
                msg.header = Header()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'system_status_input'
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
                msg = Init.Request()
                msg.header = Header()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'system_status_input'
                msg.vehicle_id = vehicle
                msg.start = Bool(data=start_config["start_node"])
                msg.rosbag_flag = Bool(data=start_config["record_rosbag"])
                msg.rosbag_prefix = start_config["rosbag_prefix"]
                msg.thruster_arm = Bool(data=start_config["arm_thruster"])
                msg.dvl_acoustics = Bool(data=start_config["start_dvl"])
                client = node.init_client
                node.get_logger().info(f"client {client}")
                if client is not None and client.wait_for_service(timeout_sec=1.0):
                    future = client.call_async(msg)
                    rclpy.spin_until_future_complete(node, future)
                    if future.result() is not None:
                        node.get_logger().info("Init command initiated successfully.")
                    else:
                        node.get_logger().error("Failed to send init command.")
                else:
                    node.get_logger().error(f"Init service for vehicle {vehicle} not available.")
    except Exception as e:
        node.get_logger().error(f"Error getting user input: {e}")