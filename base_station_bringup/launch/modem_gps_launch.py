import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros


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

    gpsd_host_launch_arg = DeclareLaunchArgument(
        'gpsd_host',
        default_value='localhost',
        description='Host running gpsd for the base station GPS receiver'
    )
    gpsd_port_launch_arg = DeclareLaunchArgument(
        'gpsd_port',
        default_value='2947',
        description='Port gpsd is listening on'
    )

    gps_node_container = launch_ros.actions.ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='base_station_gps_container',
        namespace='',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='gpsd_client',
                plugin='gpsd_client::GPSDClientComponent',
                name='gpsd_client',
                namespace='',
                parameters=[{
                    'host': LaunchConfiguration('gpsd_host'),
                    'port': LaunchConfiguration('gpsd_port'),
                    'use_gps_time': False,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='log',
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    return LaunchDescription([
        param_file_arg,
        modem_pinger,
        seatrac_node,
        mapviz_launch,
        gpsd_host_launch_arg,
        gpsd_port_launch_arg,
        gps_node_container,
    ])
