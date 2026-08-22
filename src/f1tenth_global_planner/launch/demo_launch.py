import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_global_planner')
    map_yaml_path = os.path.join(pkg_share, 'maps', 'F1tenth_Map_walled.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    global_planner_node = Node(
        package='f1tenth_global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[{
            'start_x': 0.496,
            'start_y': -2.748,
            'goal_x': 0.617,
            'goal_y': -1.443,
            'robot_radius': 0.20,
            'use_rviz_goals': True,
        }]
    )

    path_smoother_node = Node(
        package='f1tenth_global_planner',
        executable='path_smoother_node',
        name='path_smoother_node',
        output='screen',
        parameters=[{
            'points_per_meter': 10.0,
            'safety_radius': 0.08,
        }]
    )

    sim_tf_broadcaster_node = Node(
        package='f1tenth_global_planner',
        executable='sim_tf_broadcaster_node',
        name='sim_tf_broadcaster_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node,
        global_planner_node,
        path_smoother_node,
        sim_tf_broadcaster_node,
        rviz_node,
    ])