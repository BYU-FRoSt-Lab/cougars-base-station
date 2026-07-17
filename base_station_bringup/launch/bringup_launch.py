import os
from pathlib import Path

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros


def generate_launch_description():

    ### Launch arguments
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=f'{Path.home()}/config/base/base_station_params.yaml'
    )
    use_coms_launch_arg = DeclareLaunchArgument(
        'use_coms',
        default_value='true',
        description='Launch base station communications'
    )
    use_gui_launch_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch base station GUI'
    )
    use_mapviz_launch_arg = DeclareLaunchArgument(
        'use_mapviz',
        default_value='true',
        description='Launch base station map visualization'
    )
    acoms_on_launch_arg = DeclareLaunchArgument(
        'acoms_on',
        default_value='true',
        description='Launch Seatrac acoustic modem node'
    )

    launch_args = [
        ('param_file', LaunchConfiguration('param_file')),
    ]

    ### Package launch directories
    coms_dir = os.path.join(
        get_package_share_directory('base_station_coms'), 'launch')
    gui_dir = os.path.join(
        get_package_share_directory('base_station_gui'), 'launch')
    mapviz_dir = os.path.join(
        get_package_share_directory('cougars_mapviz'), 'launch')

    ### Launch files
    coms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coms_dir, 'coms_launch.py')),
        launch_arguments=launch_args,
        condition=IfCondition(LaunchConfiguration('use_coms')))

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gui_dir, 'gui_launch.py')),
        launch_arguments=launch_args,
        condition=IfCondition(LaunchConfiguration('use_gui')))

    mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapviz_dir, 'mapviz_launch.py')),
        launch_arguments=launch_args,
        condition=IfCondition(LaunchConfiguration('use_mapviz')))
    
    # seatrac_node = launch_ros.actions.Node(
    #     package='seatrac',
    #     executable='modem',
    #     parameters=[LaunchConfiguration('param_file')],
    #     output='log',
    #     condition=IfCondition(LaunchConfiguration('acoms_on')),
    # )

    


    launch_actions = [
        # launch args
        param_file_launch_arg,
        use_coms_launch_arg,
        use_gui_launch_arg,
        use_mapviz_launch_arg,
        acoms_on_launch_arg,
        # launch files
        coms_launch,
        gui_launch,
        mapviz_launch,
        # seatrac_node,
    ]

    return launch.LaunchDescription(launch_actions)
