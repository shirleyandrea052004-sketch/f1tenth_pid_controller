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
    pkg_share = get_package_share_directory('f1tenth_pid_controller')
    default_params = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    default_csv = os.path.join(_default_waypoints_dir(), 'smoothed_path.csv')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML de parámetros del controlador PID.',
    )

    waypoints_csv_arg = DeclareLaunchArgument(
        'waypoints_csv_path',
        default_value=default_csv,
        description=(
            'CSV de waypoints a seguir. Debe coincidir con el que genera '
            'demo_launch.py.'),
    )

    return LaunchDescription([
        params_arg,
        waypoints_csv_arg,
        Node(
            package='f1tenth_pid_controller',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            # El segundo diccionario se aplica DESPUÉS del YAML, así que la
            # ruta resuelta aquí tiene prioridad sobre la que traiga el
            # archivo de parámetros. Así el controlador nunca queda apuntando
            # a un CSV viejo por una ruta absoluta escrita a mano.
            parameters=[
                LaunchConfiguration('params_file'),
                {'path_csv': LaunchConfiguration('waypoints_csv_path')},
            ],
        ),
    ])