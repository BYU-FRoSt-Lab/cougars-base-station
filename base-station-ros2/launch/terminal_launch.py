
import sys
import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # COLCON_PREFIX_PATH is the path to the root folder of the sourced ros workspace
    ros_ws_path = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)

    param_file = os.path.join(ros_ws_path, 'base_station_params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            name='modem',
            parameters=[param_file]
        ),
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_coms',
            name='base_station_coms',
            parameters=[param_file]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()

