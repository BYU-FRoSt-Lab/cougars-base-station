
import sys
import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the first path in COLCON_PREFIX_PATH
    colcon_prefix_path = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)
    if not colcon_prefix_path:
        raise RuntimeError("COLCON_PREFIX_PATH is not set or empty. Make sure your workspace is sourced.")
    # The first entry is typically the current workspace's 'install' directory
    workspace_install_dir = colcon_prefix_path[0]
    # Traverse up to get the workspace root (assuming standard layout)
    workspace_root = os.path.abspath(os.path.join(workspace_install_dir, '..'))

    param_file = os.path.join(workspace_root, 'base_station_params.yaml')

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
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_modem',
            name='base_station_modem',
            parameters=[param_file]
        ),
        launch_ros.actions.Node(
            package='base_station_coms',
            executable='base_station_radio.py',
            name='base_station_radio',
            parameters=[param_file]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()

