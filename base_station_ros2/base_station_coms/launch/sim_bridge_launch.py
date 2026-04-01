"""
sim_bridge_launch.py

Launches two radio_bridge nodes cross-wired over ROS topics to simulate
a base station and an AUV communicating over a radio link — no hardware needed.

    Node A (device_id=1): TX=/a_to_b  RX=/b_to_a
    Node B (device_id=2): TX=/b_to_a  RX=/a_to_b

Usage:
    ros2 launch base_station_coms sim_bridge_launch.py
    ros2 launch base_station_coms sim_bridge_launch.py config_file:=/path/to/bridge.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join("/home", "frostlab", "config", "bridge.yaml"),
        description="Path to the bridge.yaml topic configuration file",
    )

    config_file = LaunchConfiguration("config_file")

    node_a = Node(
        package="base_station_coms",
        executable="radio_bridge.py",
        name="rf_bridge_a",
        parameters=[{
            "config_file":    config_file,
            "sim_mode":       True,
            "sim_tx_topic":   "/radio_sim/a_to_b",
            "sim_rx_topic":   "/radio_sim/b_to_a",
            "device_id":      1,
        }],
        output="screen",
        emulate_tty=True,
    )

    node_b = Node(
        package="base_station_coms",
        executable="radio_bridge.py",
        name="rf_bridge_b",
        parameters=[{
            "config_file":    config_file,
            "sim_mode":       True,
            "sim_tx_topic":   "/radio_sim/b_to_a",
            "sim_rx_topic":   "/radio_sim/a_to_b",
            "device_id":      2,
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        node_a,
        node_b,
    ])
