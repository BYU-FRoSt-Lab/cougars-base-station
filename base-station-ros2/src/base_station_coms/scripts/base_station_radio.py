#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from base_station_interfaces.msg import Status
from base_station_interfaces.msg import Connections
from base_station_interfaces.srv import BeaconId
from std_msgs.msg import String
from std_msgs.msg import Bool
import time


from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.exception import TransmitException

import json
import threading
import traceback

class RFBridge(Node):
    def __init__(self):
        super().__init__('base_station_rf_bridge')

        # Namespace logic not needed if you use the launch file

        # vehicle_ns = ''
        # try:
        #     params = self.get_node_parameters_interface().get_parameter_overrides()
        #     vehicle_namespaces = [key.split('.')[0] for key in params.keys()
        #                           if key.startswith('coug') and not key.startswith('/**')]
        #     if vehicle_namespaces:
        #         vehicle_ns = vehicle_namespaces[0]
        #         self.get_logger().info(f"Found vehicle namespace from params: {vehicle_ns}")
        # except Exception as e:
        #     self.get_logger().debug(f"Couldn't get namespace from parameters: {str(e)}")
        # if not vehicle_ns:
        #     node_namespace = self.get_namespace()
        #     if node_namespace and node_namespace != '/':
        #         vehicle_ns = node_namespace.strip('/')
        #         # self.get_logger().info(f"Using parent namespace: {vehicle_ns}")
        # if not vehicle_ns:
        #     vehicle_ns = self.declare_parameter('namespace', '').value
        #     if vehicle_ns:
        #         self.get_logger().info(f"Using explicitly provided namespace: {vehicle_ns}")
        # self.namespace = vehicle_ns.strip('/')
        # if self.namespace and not self.namespace.endswith('/'):
        #     self.namespace += '/'
        # # self.get_logger().info(f"Final vehicle namespace: '{self.namespace}'")

        # Debug mode
        self.debug_mode = self.declare_parameter('debug_mode', False).value
        if self.debug_mode:
            self.get_logger().info("Debug mode enabled: Will log detailed packet information")

        # QoS profiles
        self.odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.dvl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        # Data storage
        # self.latest_status_data = "NO_DATA"
        # self.latest_odom = "NO_DATA"
        # self.latest_leak = "NO_DATA"
        # self.latest_battery = "NO_DATA"
        # self.latest_dvl_velocity = "NO_DATA"
        # self.latest_dvl_position = "NO_DATA"
        self.declare_parameter('vehicles_in_mission', [1,2,3])
        self.vehicles_in_mission = self.get_parameter('vehicles_in_mission').get_parameter_value().integer_array_value

        self.declare_parameter('ping_frequency', 2)
        self.ping_frequency = self.get_parameter('ping_frequency').get_parameter_value().integer_value

        self.declare_parameter('vehicle_id', 15)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

        #XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/frost/xbee_radio').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)

        # ROS publishers and subscribers
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        self.init_publisher = self.create_publisher(String, 'init', 10)
        self.status_publisher = self.create_publisher(Status, 'status', 10)
        self.rf_connection_publisher = self.create_publisher(Connections, 'connections', 10)
        self.confirm_e_kill_publisher = self.create_publisher(Bool, 'confirm_e_kill', 10)

        self.e_kill_service = self.create_service(BeaconId, 'radio_e_kill', self.send_e_kill_callback)
        self.status_service = self.create_service(BeaconId, 'radio_status_request', self.request_status_callback)

        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)
        
        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
                    # Register XBee data receive callback
            self.device.add_data_received_callback(self.data_receive_callback)
            self.get_logger().info("RF Bridge node started using digi-xbee library.")

            self.timer = self.create_timer(self.ping_frequency, self.check_connections)


            # Thread-safe shutdown flag
            self.running = True
            self.ping_timestamp = {}
            self.connections = {}
            self.radio_addresses = {}
            self.max_msgs_missed = 3  # Number of missed messages before considering a vehicle disconnected
            for vehicle in self.vehicles_in_mission:
                self.connections[vehicle] = False
                self.ping_timestamp[vehicle] = 0
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")





    def tx_callback(self, msg):
        try:
            message = msg.data
            self.device.send_data_broadcast(message)
            self.get_logger().debug(f"Sent via XBee: {message}")
        except Exception as e:
            self.get_logger().error(f"XBee transmission error: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def send_message(self, msg, address):
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, msg)
            self.get_logger().debug(f"Sent via XBee: {msg}")
            return True
        except TransmitException as e:
            self.get_logger().error(f"XBee transmission error - TransmitException: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
        except Exception as e:
            self.get_logger().error(f"XBee transmission error - Exception: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            return False


    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data.decode('utf-8', errors='replace')
            sender_address = xbee_message.remote_device.get_64bit_addr()
            self.get_logger().debug(f"Received from {sender_address}: {payload}")

            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                data = payload  # If JSON decoding fails, treat payload as a string

            if isinstance(data, dict):
                message_type = data.get("message")
            else:
                message_type = data

            if message_type == "STATUS":
                self.recieve_status(data)
            elif message_type == "E_KILL":
                self.confirm_e_kill(data)
            elif message_type == "PING":
                self.recieve_ping(data, sender_address)
            else:
                self.get_logger().warn(f"Unknown message type: {message_type}")
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")




    def check_connections(self):
        msg = Connections()
        msg.connection_type = 1
        self.get_logger().debug(f"Sending PING")
        ping = "PING"
        if len(self.radio_addresses) < len(self.vehicles_in_mission):
            self.device.send_data_broadcast(ping)
        else:
            for vehicle in self.radio_addresses:
                self.send_message(ping, self.radio_addresses[vehicle])
                last_ping = int(time.time() - self.ping_timestamp[vehicle])
                if (last_ping >= self.ping_frequency*self.max_msgs_missed):
                    self.connections[vehicle] = False 
                msg.connections.append(self.connections[vehicle])
                msg.last_ping.append(last_ping)
                self.rf_connection_publisher.publish(msg)

        if self.debug_mode:
            self.get_logger().debug(f"Connections: {self.connections}")


    def request_status_callback(self, request, response):

        try:
            target_vehicle_id = request.beacon_id
            if target_vehicle_id is None:
                self.get_logger().error("Status request missing target vehicle ID.")
                response.success = False
                return response

            self.get_logger().debug(f"Received status request for Coug {target_vehicle_id}")

            status_request_msg = "STATUS"

            response.success = self.send_message(status_request_msg, self.radio_addresses.get(target_vehicle_id, None))
        except Exception as e:
            self.get_logger().error(f"Error processing status request: {e}")
            response.success = False

        return response

    def recieve_status(self, data):
        self.get_logger().info(f"Coug {data.get('src_id', 'unknown')}'s Status:")
        self.get_logger().info(f"    Data: {data}")

        status = Status()
        status.vehicle_id = data.get('src_id', 0)
        status.x = data.get('x', 0)
        status.y = data.get('y', 0)
        status.heading = data.get('heading', 0)
        status.dvl_vel = data.get('dvl_vel', 0)
        status.battery_voltage = data.get('battery', 0)
        status.dvl_running = data.get('dvl_running', False)
        status.gps_connection = data.get('gps_connection', False)
        status.leak_detection = data.get('leak_detection', False)
        self.status_publisher.publish(status)




    def recieve_ping(self, msg, sender_address):
        self.radio_addresses[msg["src_id"]] = sender_address
        vehicle_id = msg["src_id"]
        self.ping_timestamp[vehicle_id] = time.time()
        self.connections[vehicle_id] = True


    def send_e_kill_callback(self, request, response):
        try:
            target_vehicle_id = request.beacon_id
            if target_vehicle_id is None:
                self.get_logger().error("Emergency kill request missing target vehicle ID.")
                response.success = False
                return response

            self.get_logger().debug(f"Received emergency kill request for Coug {target_vehicle_id}")

            e_kill_msg = "E_KILL"

            self.send_message(e_kill_msg, self.radio_addresses.get(target_vehicle_id, None))
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error processing emergency kill request: {e}")
            response.success = False

        return response

    def confirm_e_kill(self, data):
        if data.get("success"):
            self.get_logger().debug(f"Emergency kill command was successful for Coug {data.get('src_id')}")
        else:
            self.get_logger().warn(f"Emergency kill command failed for Coug {data.get('src_id')}")

        success_msg = Bool()
        success_msg.data = data.get("success", False)
        self.confirm_e_kill_publisher.publish(success_msg)




    def destroy_node(self):
        self.running = False
        if self.device is not None and self.device.is_open():
            self.device.close()
            self.get_logger().info("XBee device closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RFBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down RF Bridge node.")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
