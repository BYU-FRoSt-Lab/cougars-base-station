import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

#######################################
#
# Launches the base station GPS fix source (gpsd_client) needed for the
# comm_range_test.py range test.
#
# comm_range_test.py is NOT launched here: ros2 launch manages child
# processes over asyncio and does not forward the terminal's stdin to
# them, so its interactive input() prompts would hang forever with no
# way to type into them. Launch this file first, then in a second
# terminal (with the workspace sourced) run:
#   ros2 run base_station_coms comm_range_test.py
#
#######################################


def generate_launch_description():

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

    launch_actions = [
        gpsd_host_launch_arg,
        gpsd_port_launch_arg,
        gps_node_container,
    ]

    return launch.LaunchDescription(launch_actions)
