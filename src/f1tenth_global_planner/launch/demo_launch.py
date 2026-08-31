import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_waypoints_dir():
    """Resuelve dónde leer/guardar los waypoints generados.

    Orden de resolución:
      1. F1TENTH_WAYPOINTS_DIR, si está definida (permite forzar la ruta).
      2. ~/f1tenth-dijkstra-global-planner/waypoints — la carpeta del
         repositorio del planificador, para que el CSV quede versionado
         junto al código que lo genera.
      3. <raíz del workspace colcon>/waypoints, si la anterior no existe.
      4. ~/f1tenth_waypoints como último recurso.
    """
    env_dir = os.environ.get('F1TENTH_WAYPOINTS_DIR')
    if env_dir:
        return env_dir

    repo_dir = os.path.expanduser('~/f1tenth-dijkstra-global-planner/waypoints')
    if os.path.isdir(os.path.dirname(repo_dir)):
        return repo_dir

    # COLCON_PREFIX_PATH apunta a <workspace>/install, así que su directorio
    # padre es la raíz del workspace (donde viven src/ y waypoints/).
    prefix_path = os.environ.get('COLCON_PREFIX_PATH', '')
    for prefix in prefix_path.split(os.pathsep):
        if not prefix:
            continue
        workspace = os.path.dirname(prefix)
        if os.path.isdir(os.path.join(workspace, 'src')):
            return os.path.join(workspace, 'waypoints')

    return os.path.expanduser('~/f1tenth_waypoints')


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_global_planner')
    map_yaml_path = os.path.join(pkg_share, 'maps', 'F1tenth_Map_walled.yaml')

    default_csv = os.path.join(_default_waypoints_dir(), 'smoothed_path.csv')

    waypoints_csv_arg = DeclareLaunchArgument(
        'waypoints_csv_path',
        default_value=default_csv,
        description=(
            'Ruta del CSV de waypoints suavizados. Por defecto se guarda en '
            'la carpeta waypoints/ del workspace para que quede versionada '
            'junto al repositorio.'),
    )

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
            'start_x': 0.792,
            'start_y': 3.535,
            'goal_x': 0.600,
            'goal_y': 4.701,
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
            'waypoints_csv_path': LaunchConfiguration('waypoints_csv_path'),
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
        waypoints_csv_arg,
        map_server_node,
        lifecycle_manager_node,
        global_planner_node,
        path_smoother_node,
        sim_tf_broadcaster_node,
        rviz_node,
    ])