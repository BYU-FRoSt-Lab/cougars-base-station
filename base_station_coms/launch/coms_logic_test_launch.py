import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join('/home/frostlab/config', 'base_station_params.yaml'),
        description='Path to the base station parameter YAML file'
    )
    param_file = LaunchConfiguration('param_file')

    quiet_test_params = {
        'disable_auto_link_status': True,
    }

    return LaunchDescription([
        param_file_arg,
        Node(
            package='base_station_coms',
            executable='base_station_wifi.py',
            name='base_station_wifi',
            parameters=[param_file, quiet_test_params],
            output='screen'
        ),
        Node(
            package='base_station_coms',
            executable='base_station_modem',
            name='base_station_modem',
            parameters=[param_file, quiet_test_params],
            output='screen'
        ),
        Node(
            package='base_station_coms',
            executable='base_station_radio.py',
            name='base_station_radio',
            parameters=[param_file, quiet_test_params],
            output='screen'
        ),
        Node(
            package='base_station_coms',
            executable='coms_logic_test.py',
            name='coms_logic_test',
            parameters=[param_file],
            output='screen'
        ),
    ])
