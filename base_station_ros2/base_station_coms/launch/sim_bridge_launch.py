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
# get package share directory
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=os.path.join(get_package_share_directory("base_station_coms"), "config", "radio_bridge_params.yaml"),
        description="Path to the parameters file",
    )

    param_file = LaunchConfiguration("param_file")

    node_a = Node(
        package="base_station_coms",
        executable="radio_bridge.py",
        name="rf_bridge_a",
        parameters=[param_file],
        output="screen",
        emulate_tty=True,
    )

    node_b = Node(
        package="base_station_coms",
        executable="radio_bridge.py",
        name="rf_bridge_b",
        parameters=[param_file],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        param_file_arg,
        node_a,
        node_b,
    ])
