import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_pid_controller')
    default_params = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    return LaunchDescription([
        Node(
            package='f1tenth_pid_controller',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[default_params],
        ),
    ])
