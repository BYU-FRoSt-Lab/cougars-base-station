import launch
import launch_ros.actions
from datetime import datetime

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='data_reader',
            executable='read',
            name='read'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-r', '200', './rosbag2_2024_11_09-11_59_09']
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()