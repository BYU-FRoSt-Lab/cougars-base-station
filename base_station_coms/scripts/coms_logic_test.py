#!/usr/bin/env python3

import itertools

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus
from geographic_msgs.msg import RouteNetwork
from cougars_interfaces.msg import SystemControl
from seatrac_interfaces.msg import ModemSend
from std_msgs.msg import Bool


class ComsLogicTest(Node):
    def __init__(self):
        super().__init__('coms_logic_test')

        self.declare_parameter('vehicles_in_mission', [1])
        self.declare_parameter('step_period_seconds', 2.0)
        self.vehicles_in_mission = list(self.get_parameter('vehicles_in_mission').value)
        self.step_period_seconds = self.get_parameter('step_period_seconds').value

        self.status_publishers = {
            vehicle: self.create_publisher(DiagnosticStatus, f'coug{vehicle}/link_status', 10)
            for vehicle in self.vehicles_in_mission
        }
        self.load_mission_publishers = {
            vehicle: self.create_publisher(RouteNetwork, f'coug{vehicle}/load_mission', 10)
            for vehicle in self.vehicles_in_mission
        }
        self.start_mission_publishers = {
            vehicle: self.create_publisher(SystemControl, f'coug{vehicle}/start_mission', 10)
            for vehicle in self.vehicles_in_mission
        }
        self.emergency_kill_publishers = {
            vehicle: self.create_publisher(Bool, f'coug{vehicle}/emergency_kill', 10)
            for vehicle in self.vehicles_in_mission
        }
        self.mission_observers = [
            self.create_subscription(
                RouteNetwork,
                f'coug{vehicle}/mission',
                lambda msg, vehicle=vehicle: self.get_logger().info(
                    f"Observed WiFi mission publish for Coug {vehicle}"
                ),
                10
            )
            for vehicle in self.vehicles_in_mission
        ]
        self.start_observers = [
            self.create_subscription(
                SystemControl,
                f'coug{vehicle}/system/status',
                lambda msg, vehicle=vehicle: self.get_logger().info(
                    f"Observed WiFi start publish for Coug {vehicle}"
                ),
                10
            )
            for vehicle in self.vehicles_in_mission
        ]

        self.modem_send_subscriber = self.create_subscription(
            ModemSend,
            'modem_send',
            self.modem_send_callback,
            10
        )

        self.scenarios = list(itertools.product([False, True], repeat=3))
        self.scenario_index = 0
        self.phase = 'status'
        self.wait_cycles_after_status = 0
        self.timer = self.create_timer(self.step_period_seconds, self.step)

        self.get_logger().info(
            "Comms logic test started. Scenarios are ordered as wifi, radio, modem booleans."
        )

    def step(self):
        if self.scenario_index >= len(self.scenarios):
            self.get_logger().info("Comms logic test complete.")
            self.timer.cancel()
            return

        wifi_ok, radio_ok, modem_ok = self.scenarios[self.scenario_index]

        if self.phase == 'status':
            self.publish_status_scenario(wifi_ok, radio_ok, modem_ok)
            self.wait_cycles_after_status = 1
            self.phase = 'commands'
            return

        if self.wait_cycles_after_status > 0:
            self.wait_cycles_after_status -= 1
            return

        self.publish_commands(wifi_ok, radio_ok, modem_ok)
        self.phase = 'status'
        self.scenario_index += 1

    def publish_status_scenario(self, wifi_ok, radio_ok, modem_ok):
        self.get_logger().info(
            f"Scenario {self.scenario_index + 1}/{len(self.scenarios)}: "
            f"wifi={wifi_ok}, radio={radio_ok}, modem={modem_ok}"
        )

        for vehicle in self.vehicles_in_mission:
            publisher = self.status_publishers[vehicle]
            publisher.publish(self.make_status(vehicle, 'wifi', wifi_ok))
            publisher.publish(self.make_status(vehicle, 'radio', radio_ok))
            publisher.publish(self.make_status(vehicle, 'modem', modem_ok))

    def publish_commands(self, wifi_ok, radio_ok, modem_ok):
        del wifi_ok, radio_ok, modem_ok

        for vehicle in self.vehicles_in_mission:
            self.get_logger().info(f"Publishing test commands for Coug {vehicle}")
            self.load_mission_publishers[vehicle].publish(RouteNetwork())
            self.start_mission_publishers[vehicle].publish(self.make_start_command(vehicle))
            self.emergency_kill_publishers[vehicle].publish(Bool(data=True))

    def make_status(self, vehicle, hardware_id, connected):
        msg = DiagnosticStatus()
        msg.name = f"Test Coug{vehicle} {hardware_id} Connection"
        msg.hardware_id = hardware_id
        msg.level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
        msg.message = "Connected" if connected else "Disconnected"
        return msg

    def make_start_command(self, vehicle):
        msg = SystemControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'coms_logic_test'
        msg.start.data = True
        msg.rosbag_flag.data = False
        msg.rosbag_prefix = f'test_coug{vehicle}'
        msg.thruster_arm.data = False
        msg.dvl_acoustics.data = False
        return msg

    def modem_send_callback(self, msg):
        self.get_logger().info(
            f"Observed modem_send: dest_id={msg.dest_id}, packet_len={msg.packet_len}, msg_type={msg.msg_type}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ComsLogicTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
