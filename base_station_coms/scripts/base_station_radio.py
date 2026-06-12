#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from base_station_interfaces.msg import ConsoleLog,
from cougars_interfaces.msg import MissionFeedback, SystemControl, WaypointFeedback
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from dvl_msgs.msg import DVL
from geographic_msgs.msg import RouteNetwork
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from std_msgs.msg import String, Bool
from std_msgs.msg import Int8
import time
import math


from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.exception import TransmitException

import json
import traceback
from pathlib import Path
import base64

class VehicleRadioConnection:
    def __init__(self, vehicle_id, node, max_missed_messages):
        self.vehicle_id = vehicle_id
        self.node = node
        self.max_missed_messages = max_missed_messages
        self.connection_status = False
        self.radio_address = None
        self.last_ping_time = time.time()
        self.last_status_request_time = 0.0
        self.modem_connection = False
        self.wifi_connection = False
    

        self.link_status_publisher = node.create_publisher(
            DiagnosticStatus,
            f'coug{vehicle_id}/link_status',
            10
        )

        self.state_estimate_publisher = node.create_publisher(
            Odometry,
            f'coug{vehicle_id}/state_estimate',
            10
        )

        self.pressure_data_publisher = node.create_publisher(
            FluidPressure,
            f'coug{vehicle_id}/pressure_data',
            10
        )

        self.battery_data_publisher = node.create_publisher(
            BatteryState,
            f'coug{vehicle_id}/battery_data',
            10
        )

        self.dvl_publisher = node.create_publisher(
            DVL,
            f'coug{vehicle_id}/dvl',
            10
        )

        self.waypoint_feedback_publisher = node.create_publisher(
            WaypointFeedback,
            f'coug{vehicle_id}/waypoint_feedback',
            10
        )

        self.link_status_subscriber = node.create_subscription(
            DiagnosticStatus,
            f'coug{vehicle_id}/link_status',
            self.link_status_callback,
            10
        )

        self.load_mission_subscriber = node.create_subscription(
            RouteNetwork,
            f'coug{vehicle_id}/load_mission',
            self.load_mission_callback,
            10
        )

        self.start_mission_subscriber = node.create_subscription(
            SystemControl,
            f'coug{vehicle_id}/start_mission',
            self.start_mission_callback,
            10
        )

        self.emergency_kill_subscriber = node.create_subscription(
            Bool,
            f'coug{vehicle_id}/emergency_kill',
            self.emergency_kill_callback,
            10
        )

        self.emergency_surface_subscriber = node.create_subscription(
            Bool,
            f'coug{vehicle_id}/emergency_surface',
            self.emergency_surface_callback,
            10
        )

    def link_status_callback(self, msg):
        connected = msg.level == DiagnosticStatus.OK
        if msg.hardware_id == "radio":
            self.connection_status = connected
        elif msg.hardware_id == "wifi":
            self.wifi_connection = connected
        elif msg.hardware_id == "modem":
            self.modem_connection = connected

    def load_mission_callback(self, msg):
        if not self.wifi_connection and self.connection_status:
            # log sending over radio
            self.node.get_logger().info(f"Sending mission for vehicle {self.vehicle_id} over radio")

    def start_mission_callback(self, msg):
        if not self.wifi_connection and self.connection_status:
            # log sending over radio
            self.node.get_logger().info(f"Starting mission for vehicle {self.vehicle_id} over radio")

    def emergency_kill_callback(self, msg):
        if msg.data and not self.wifi_connection and self.connection_status:
            # log sending over radio
            self.node.get_logger().info(f"Emergency kill for vehicle {self.vehicle_id} over radio")

    def emergency_surface_callback(self, msg):
        if msg.data and not self.wifi_connection and self.connection_status:
            self.node.get_logger().info(f"Emergency surface for vehicle {self.vehicle_id} over radio")

    def handle_ping(self, sender_address):
        self.radio_address = sender_address
        self.last_ping_time = time.time()
        self.connection_status = True
        self.publish_link_status()

    def has_address(self):
        return self.radio_address is not None

    def is_connected(self):
        return self.connection_status

    def seconds_since_ping(self):
        return int(time.time() - self.last_ping_time)

    def check_connection(self, ping_frequency):
        if self.seconds_since_ping() >= ping_frequency * self.max_missed_messages:
            self.connection_status = False
        self.publish_link_status()


    def should_request_status(self):
        return self.connection_status and not self.wifi_connection and self.has_address()

    def request_status(self, send_fn):
        if not self.should_request_status():
            return False

        status_request = {
            "message": "STATUS",
            "vehicle_id": self.vehicle_id,
        }
        sent = self.send_message(send_fn, json.dumps(status_request, separators=(',', ':')))
        if sent:
            self.last_status_request_time = time.time()
            self.node.get_logger().debug(f"Requested status from Coug{self.vehicle_id} over radio")
        return sent

    def publish_link_status(self):
        status_msg = DiagnosticStatus()
        status_msg.name = f"Coug{self.vehicle_id} Radio Connection"
        status_msg.hardware_id = "radio"
        status_msg.level = DiagnosticStatus.OK if self.connection_status else DiagnosticStatus.ERROR
        status_msg.message = "Connected" if self.connection_status else "Disconnected"
        status_msg.values.append(KeyValue(key="last_message_time", value=str(self.last_ping_time)))
        self.link_status_publisher.publish(status_msg)
    
    # Function to handle received status messages
    def recieve_status(self, data):
        self.get_logger().debug(f"Coug {data.get('src_id', 'unknown')}'s Status:")
        self.get_logger().debug(f"    Data: {data}")

        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.pose.pose.position.x = data.get('x', 0.0)
        odometry_msg.pose.pose.position.y = data.get('y', 0.0)
        odometry_msg.pose.pose.position.z = data.get('z', 0.0)
        odometry_msg.pose.pose.orientation.x = data.get('qx', 0.0)
        odometry_msg.pose.pose.orientation.y = data.get('qy', 0.0)
        odometry_msg.pose.pose.orientation.z = data.get('qz', 0.0)
        odometry_msg.pose.pose.orientation.w = data.get('qw', 1.0)
        self.odom_publisher.publish(odometry_msg)

        fluid_pressure_msg = FluidPressure()
        fluid_pressure_msg.fluid_pressure = data.get('pressure', 0.0)
        self.pressure_publisher.publish(fluid_pressure_msg)

        battery_state_msg = BatteryState()
        battery_state_msg.voltage = data.get('voltage', 0.0)
        self.battery_publisher.publish(battery_state_msg)

        dvl_msg = DVL()
        dvl_msg.velocity.x = data.get('dvl_x', 0.0)
        dvl_msg.velocity.y = data.get('dvl_y', 0.0)
        dvl_msg.velocity.z = data.get('dvl_z', 0.0)
        self.dvl_publisher.publish(dvl_msg)

        waypoint_msg = WaypointFeedback()
        waypoint_msg.state = data.get('ws', 0)
        waypoint_msg.horizontal_distance_error = data.get('hd', 0.0)
        self.waypoint_publisher.publish(waypoint_msg)

        mission_msg = MissionFeedback()
        mission_msg.state = data.get('ms', 0)
        mission_msg.waypoints_completed = data.get('wc', 0)
        mission_msg.waypoints_total = data.get('tw', 0)
        mission_msg.elapsed_time = data.get('et', 0.0)
        mission_msg.current = data.get('c', 0.0)
        self.mission_publisher.publish(mission_msg)




class RFBridge(Node):
    def __init__(self):
        super().__init__('base_station_rf_bridge')

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

        # list of vehicles in the mission
        self.declare_parameter('vehicles_in_mission', [1,2,3])
        self.vehicles_in_mission = self.get_parameter('vehicles_in_mission').get_parameter_value().integer_array_value

        # Frequency of PING messages keeping track of radio connections
        self.declare_parameter('ping_frequency', 2)
        self.ping_frequency = self.get_parameter('ping_frequency').get_parameter_value().integer_value
        self.request_status = self.declare_parameter('request_status', True).value
        self.status_request_frequency_seconds = self.declare_parameter(
            'status_request_frequency_seconds',
            2.0
        ).value

        # base station vehicle ID
        self.declare_parameter('vehicle_id', 15)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

        #XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/frost/xbee_radio').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)

        self.radio_addresses = {}

        # publishes console log messages to GUI
        self.print_to_gui_publisher = self.create_publisher(ConsoleLog, 'console_log', 10)
    
        self.running = True
        self.max_msgs_missed = 5  # Number of missed messages before considering a vehicle disconnected
        self.vehicle_radios = {
            vehicle: VehicleRadioConnection(vehicle, self, self.max_msgs_missed)
            for vehicle in self.vehicles_in_mission
        }
    
        self.timer = self.create_timer(self.ping_frequency, self.check_connections)

        if self.request_status:
            self.status_request_timer = self.create_timer(
                self.status_request_frequency_seconds,
                self.request_status_when_wifi_disconnected
            )
        else:
            self.get_logger().info("Radio status requests disabled by parameter")

        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
                    # Register XBee data receive callback
            self.device.add_data_received_callback(self.data_receive_callback)
            self.get_logger().info("RF Bridge node started using digi-xbee library.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")

    # Function to send a message to a specific address
    def send_message(self, msg, address):
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, msg)
            self.get_logger().debug(f"Sent via XBee: {msg}")
            return True
        except TransmitException as e:
            self.get_logger().debug(f"XBee transmission error - TransmitException: {e}")
            self.get_logger().debug(traceback.format_exc())
            return False
        except Exception as e:
            self.get_logger().debug(f"XBee transmission error - Exception: {str(e)}")
            self.get_logger().debug(traceback.format_exc())
            return False

    # Callback for receiving data from XBee
    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data.decode('utf-8', errors='replace')
            sender_address = xbee_message.remote_device.get_64bit_addr()

            self.get_logger().debug(f"Received from {sender_address}: {payload}")

            data = json.loads(payload)
            message_type = data.get("message")
            if (message_type == "PING"):
                self.get_logger().debug(f"Received PING from vehicle {data.get('src_id')}")
                self.recieve_ping(data.get("src_id"), sender_address)
            
            if sender_address not in self.radio_addresses:
                self.get_logger().warn(f"Received message from unknown vehicle: {sender_address}")
                return


            if message_type == "STATUS":
                self.vehicle_radios[self.radio_addresses[sender_address]].recieve_status(data)
            elif message_type == "E_KILL":
                self.vehicle_radios[self.radio_addresses[sender_address]].confirm_e_kill(data)
            elif message_type == "INIT":
                self.print_to_gui_publisher.publish(ConsoleLog(message="Start mission command was successful", vehicle_number=data.get("src_id")))
            elif message_type == "INIT_ACK":
                self.get_logger().debug(f"Received INIT_ACK from vehicle {data.get('src_id')}: {'success' if data.get('success') else 'failure'}")
                self.print_to_gui_publisher.publish(ConsoleLog(message=f"Initialization of Coug {data.get('src_id')} was {'successful' if data.get('success') else 'unsuccessful'}", vehicle_number=data.get('src_id', 0)))
            elif message_type == "FILE_ACK":
                # Handle file transfer acknowledgments
                self.get_logger().debug(f"Received file transfer ACK from vehicle {data.get('src_id')}: {data.get('status', 'unknown')}")
                if data.get("status") == "error":
                    self.print_to_gui_publisher.publish(
                        ConsoleLog(
                            message=f"File transfer failed on vehicle {data.get('src_id')}: {data.get('error', 'unknown error')}",
                            vehicle_number=data.get('src_id', 0)
                        )
                    )
            else:
                self.get_logger().warn(f"Unknown message type: {message_type}")
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")



    # Function to check connections and send PING messages
    def check_connections(self):
    
        self.get_logger().debug(f"Sending PING")
        ping = "PING"

        if any(not vehicle_radio.has_address() for vehicle_radio in self.vehicle_radios.values()):
            try:
                self.device.send_data_broadcast(ping)
            except Exception as e:
                self.get_logger().debug(f"Failed to send broadcast PING: {e}")
        else:
            for vehicle_radio in self.vehicle_radios.values():
                vehicle_radio.send_message(self.send_message, ping)

        for vehicle_radio in self.vehicle_radios.values():
            vehicle_radio.check_connection(self.ping_frequency)

    def request_status_when_wifi_disconnected(self):
        for vehicle_radio in self.vehicle_radios.values():
            vehicle_radio.request_status(self.send_message)
    


    def send_file_over_radio(self, file_path, target_vehicle_id, remote_filename):
        """
        Send a file over XBee radio by chunking it into smaller packets.
        
        Args:
            file_path (str): Path to the file to send
            target_vehicle_id (int): Target vehicle ID
            remote_filename (str): Name to save the file as on the remote end
            
        Returns:
            bool: True if file sent successfully, False otherwise
        """
        try:
            # XBee has a maximum payload size, typically around 100 bytes for reliable transmission
            # We'll use smaller chunks to account for JSON overhead and ensure reliability
            CHUNK_SIZE = 64
            
            # Read the file
            with open(file_path, 'rb') as f:
                file_data = f.read()
            
            # Calculate total chunks
            total_chunks = (len(file_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
            file_size = len(file_data)

            self.print_to_gui_publisher.publish(ConsoleLog(message=f"Sending file {file_path} ({file_size} bytes) in {total_chunks} chunks to vehicle {target_vehicle_id}", vehicle_number=target_vehicle_id))

            # Send file start message
            start_msg = {
                "message": "FILE_START",
                "filename": remote_filename,
                "total_chunks": total_chunks,
                "file_size": file_size,
                "transfer_id": int(time.time())  # Use timestamp as transfer ID
            }
            
            vehicle_radio = self.vehicle_radios[target_vehicle_id]
            if not vehicle_radio.send_message(self.send_message, json.dumps(start_msg)):
                self.get_logger().error("Failed to send FILE_START message")
                return False
            
            # Wait a bit for the receiver to prepare
            time.sleep(0.1)
            
            # Send file chunks
            for chunk_num in range(total_chunks):
                start_idx = chunk_num * CHUNK_SIZE
                end_idx = min(start_idx + CHUNK_SIZE, len(file_data))
                chunk_data = file_data[start_idx:end_idx]
                
                # Encode chunk data as base64 for JSON transmission
                chunk_b64 = base64.b64encode(chunk_data).decode('ascii')
                
                chunk_msg = {
                    "message": "FILE_CHUNK",
                    "transfer_id": start_msg["transfer_id"],
                    "chunk_num": chunk_num,
                    "total_chunks": total_chunks,
                    "data": chunk_b64
                }
                
                # Send chunk with retry logic
                retry_count = 0
                max_retries = 3
                while retry_count < max_retries:
                    if vehicle_radio.send_message(self.send_message, json.dumps(chunk_msg)):
                        self.get_logger().debug(f"Sent chunk {chunk_num + 1}/{total_chunks}")
                        break
                    else:
                        retry_count += 1
                        self.get_logger().warn(f"Failed to send chunk {chunk_num + 1}/{total_chunks}, retry {retry_count}/{max_retries}")
                        time.sleep(0.1)
                
                if retry_count >= max_retries:
                    self.get_logger().error(f"Failed to send chunk {chunk_num + 1}/{total_chunks} after {max_retries} retries")
                    return False

                self.print_to_gui_publisher.publish(ConsoleLog(message=f"Sent chunk {chunk_num + 1}/{total_chunks}", vehicle_number=target_vehicle_id))

                # Small delay between chunks to avoid overwhelming the receiver
                time.sleep(0.05)
            
            # Send file end message
            end_msg = {
                "message": "FILE_END",
                "transfer_id": start_msg["transfer_id"],
                "filename": remote_filename,
                "total_chunks": total_chunks
            }
            
            if not vehicle_radio.send_message(self.send_message, json.dumps(end_msg)):
                self.get_logger().error("Failed to send FILE_END message")
                return False
            
            self.get_logger().info(f"Successfully sent file {remote_filename} to vehicle {target_vehicle_id}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending file over radio: {e}")
            return False




    # Function to handle received PING messages
    def recieve_ping(self, sender_id, sender_address):
        if sender_id not in self.vehicles_in_mission:
            self.get_logger().warn(f"Received PING from unknown vehicle ID {sender_id}. Ignoring.")
            return
        self.radio_addresses[sender_address] = sender_id
        self.vehicle_radios[sender_id].handle_ping(sender_address)

    # Function to handle node destruction
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
