import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=f'{Path.home()}/config/base/base_station_params.yaml',
    )

    mapviz_launch_dir = os.path.join(
        get_package_share_directory('cougars_mapviz'), 'launch')

    modem_pinger = Node(
        package='base_station_coms',
        executable='modem_pinger_timesync',
        name='modem_pinger_timesync',
        parameters=[LaunchConfiguration('param_file')],
        output='screen',
    )

    seatrac_node = Node(
        package='seatrac',
        executable='modem',
        parameters=[LaunchConfiguration('param_file')],
        output='log',
    )

    mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapviz_launch_dir, 'mapviz_launch.py')),
    )

    return LaunchDescription([
        param_file_arg,
        modem_pinger,
        seatrac_node,
        mapviz_launch,
    ])
