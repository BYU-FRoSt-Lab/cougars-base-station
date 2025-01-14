import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            name='modem'
        ),
        launch_ros.actions.Node(
            package='py_pinger',
            executable='py_pinger',
            name='py_pinger',
            parameters=[{'target_id': 15}]
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/modem_rec'],
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()