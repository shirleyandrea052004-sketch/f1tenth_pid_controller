"""
Monitor independiente (fuera de los nodos ROS2 del proyecto) para validar de
forma objetiva y no interactiva si el vehículo choca y cuántas vueltas
completa durante una corrida autónoma en AutoDRIVE.

Se suscribe SOLO a /autodrive/f1tenth_1/ips (no depende de /map ni tf2, para
ser un chequeo verdaderamente independiente de global_planner_node/
pid_controller_node) y compara la posición real contra el mismo mapa inflado
que usa el planificador (misma lógica de carga que pick_start_goal.py).

Uso:
    python3 scripts/collision_lap_monitor.py \
        --map-yaml src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml \
        --path-csv ~/f1tenth_waypoints/smoothed_path.csv \
        --collision-radius 0.14 --warn-radius 0.24 \
        --duration 45 --log-csv ~/f1tenth_waypoints/tuning_log.csv \
        --pid-log /path/to/pid.log --tag "kp_steer=1.0"
"""
import argparse
import csv
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from PIL import Image
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt
from std_msgs.msg import Float32


def load_map_as_occupancy(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = os.path.join(yaml_dir, meta['image'])
    img = Image.open(image_path).convert('L')
    pgm_array = np.array(img)
    ros_array = np.flipud(pgm_array)  # fila 0 = y mínima, igual que map_server
    grid = np.full(ros_array.shape, -1, dtype=np.int8)
    grid[ros_array < 50] = 100
    grid[ros_array > 200] = 0
    return grid, meta['resolution'], meta['origin']


def distance_map_meters(grid, resolution):
    occupied = (grid == 100) | (grid == -1)
    dist_cells = distance_transform_edt(~occupied)
    return dist_cells * resolution


def load_path_xy(csv_path):
    xy = []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            xy.append((float(row[1]), float(row[2])))
    return xy


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CollisionLapMonitor(Node):
    def __init__(self, args):
        super().__init__('collision_lap_monitor')
        self.args = args

        grid, resolution, origin = load_map_as_occupancy(args.map_yaml)
        self.grid = grid
        self.resolution = resolution
        self.origin_x, self.origin_y = origin[0], origin[1]
        self.height, self.width = grid.shape
        self.clearance_m = distance_map_meters(grid, resolution)

        self.path_xy = load_path_xy(os.path.expanduser(args.path_csv))
        self.start_x, self.start_y = self.path_xy[0]

        self.consecutive_collision_hits = 0
        self.collided = False
        self.collision_events = []  # (t, x, y, clearance)
        self.warn_events = 0
        self.min_clearance = float('inf')
        self.min_clearance_t = None
        self.min_clearance_xy = None

        self.away_from_start = False
        self.lap_count = 0
        self.lap_times = []
        self.race_start_t = None
        self.lap_start_t = None
        self.lap_exit_radius = args.lap_trigger_radius * args.lap_exit_radius_factor

        self.n_samples = 0
        self.t0 = time.monotonic()

        # Telemetría auxiliar (para poder imprimir velocidad/dirección
        # exactamente en el momento de un choque, sin depender de
        # correlacionar a mano con el log del nodo PID).
        self.last_steering = 0.0
        self.last_throttle = 0.0
        self._last_xy = None
        self._last_t = None
        self._speed_hist = deque(maxlen=5)
        self.speed_estimate = 0.0

        self.sub = self.create_subscription(
            Point, args.ips_topic, self._ips_callback, 10)
        self.steer_sub = self.create_subscription(
            Float32, args.steering_topic, self._steering_callback, 10)
        self.throttle_sub = self.create_subscription(
            Float32, args.throttle_topic, self._throttle_callback, 10)

    def _steering_callback(self, msg: Float32):
        self.last_steering = msg.data

    def _throttle_callback(self, msg: Float32):
        self.last_throttle = msg.data

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def _ips_callback(self, msg: Point):
        x, y = msg.x, msg.y
        now = time.monotonic()
        self.n_samples += 1

        # Velocidad estimada entre muestras sucesivas de /ips (mismo
        # criterio simple que pid_controller_node), solo para poder
        # imprimirla junto al resto de la telemetría de choque.
        if self._last_xy is not None and self._last_t is not None:
            dist = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            dt = now - self._last_t
            if dt > 1e-3:
                self._speed_hist.append(dist / dt)
                self.speed_estimate = sum(self._speed_hist) / len(self._speed_hist)
        self._last_xy = (x, y)
        self._last_t = now

        telemetry = (f'v~{self.speed_estimate:.2f}m/s steer={self.last_steering:+.2f} '
                     f'throttle={self.last_throttle:+.2f}')

        gx, gy = self.world_to_grid(x, y)
        out_of_bounds = not (0 <= gx < self.width and 0 <= gy < self.height)

        if out_of_bounds:
            clearance = -1.0
            self.collision_events.append((now - self.t0, x, y, clearance))
            self.collided = True
            self.get_logger().warn(
                f'*** FUERA DEL MAPA *** @ t={now - self.t0:.1f}s '
                f'x={x:.3f} y={y:.3f} | {telemetry}')
        else:
            clearance = float(self.clearance_m[gy, gx])
            if clearance < self.min_clearance:
                self.min_clearance = clearance
                self.min_clearance_t = now - self.t0
                self.min_clearance_xy = (x, y)

            if clearance <= self.args.collision_radius:
                self.consecutive_collision_hits += 1
                if self.consecutive_collision_hits >= self.args.debounce:
                    self.collided = True
                    self.collision_events.append((now - self.t0, x, y, clearance))
                    self.get_logger().warn(
                        f'*** COLISIÓN *** @ t={now - self.t0:.1f}s '
                        f'x={x:.3f} y={y:.3f} clearance={clearance:.3f}m | {telemetry}')
            else:
                self.consecutive_collision_hits = 0
                if clearance <= self.args.warn_radius:
                    self.warn_events += 1
                    self.get_logger().info(
                        f'cuasi-choque @ t={now - self.t0:.1f}s '
                        f'x={x:.3f} y={y:.3f} clearance={clearance:.3f}m | {telemetry}')

        self._update_lap_tracking(x, y, now)

    def _update_lap_tracking(self, x, y, now):
        dist_to_start = math.hypot(x - self.start_x, y - self.start_y)

        if self.race_start_t is None:
            self.race_start_t = now
            self.lap_start_t = now

        if not self.away_from_start:
            if dist_to_start > self.lap_exit_radius:
                self.away_from_start = True
        else:
            if dist_to_start < self.args.lap_trigger_radius:
                elapsed = now - self.lap_start_t
                if elapsed > self.args.min_lap_time:
                    self.lap_count += 1
                    self.lap_times.append(elapsed)
                    self.get_logger().info(
                        f'Vuelta {self.lap_count} (monitor) | tiempo: {elapsed:.3f}s')
                    self.lap_start_t = now
                    self.away_from_start = False


def cross_check_pid_log(pid_log_path):
    if not pid_log_path or not os.path.isfile(pid_log_path):
        return []
    laps = []
    with open(pid_log_path, 'r', errors='ignore') as f:
        for line in f:
            if 'Vuelta' in line and 'completada' in line:
                laps.append(line.strip())
    return laps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--path-csv', required=True)
    parser.add_argument('--ips-topic', default='/autodrive/f1tenth_1/ips')
    parser.add_argument('--steering-topic', default='/autodrive/f1tenth_1/steering_command')
    parser.add_argument('--throttle-topic', default='/autodrive/f1tenth_1/throttle_command')
    parser.add_argument('--collision-radius', type=float, default=0.14)
    parser.add_argument('--warn-radius', type=float, default=0.24)
    parser.add_argument('--debounce', type=int, default=2,
                         help='Lecturas consecutivas ocupadas antes de declarar colisión')
    parser.add_argument('--lap-trigger-radius', type=float, default=0.45)
    parser.add_argument('--lap-exit-radius-factor', type=float, default=4.0)
    parser.add_argument('--min-lap-time', type=float, default=8.0)
    parser.add_argument('--duration', type=float, default=45.0,
                         help='Segundos de monitoreo; 0 = indefinido (hasta Ctrl+C)')
    parser.add_argument('--log-csv', default=None,
                         help='Si se da, agrega una fila-resumen a este CSV')
    parser.add_argument('--pid-log', default=None,
                         help='Log de pid_controller_node, para comparar el conteo de vueltas')
    parser.add_argument('--tag', default='',
                         help='Etiqueta libre (ej. ganancias usadas) para el log CSV')
    args = parser.parse_args()

    rclpy.init()
    node = CollisionLapMonitor(args)

    print(f'Monitoreando {args.ips_topic} por '
          f'{"tiempo indefinido" if args.duration <= 0 else f"{args.duration:.0f}s"}... '
          f'(Ctrl+C para detener antes)')

    start = time.monotonic()
    try:
        while args.duration <= 0 or (time.monotonic() - start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    elapsed = time.monotonic() - start
    node.destroy_node()
    rclpy.shutdown()

    min_clear_str = (f'{node.min_clearance:.3f}m @ t={node.min_clearance_t:.1f}s '
                      f'xy={node.min_clearance_xy}') if node.min_clearance_xy else 'sin datos'

    print('\n=== VEREDICTO ===')
    print(f'duración: {elapsed:.1f} s | muestras ips recibidas: {node.n_samples}')
    print(f'colisión: {"SI" if node.collided else "NO"} '
          f'({len(node.collision_events)} eventos, debounce={args.debounce})')
    print(f'clearance mínimo observado: {min_clear_str}')
    print(f'cuasi-choques (<{args.warn_radius}m): {node.warn_events}')
    print(f'vueltas completadas (monitor): {node.lap_count}')
    print(f'tiempos de vuelta (monitor): '
          f'{[round(t, 3) for t in node.lap_times]}')

    pid_laps = cross_check_pid_log(args.pid_log)
    if args.pid_log:
        print(f'\nLíneas de vuelta en log del nodo PID ({args.pid_log}):')
        if pid_laps:
            for line in pid_laps:
                print(f'  {line}')
            if len(pid_laps) != node.lap_count:
                print(f'  *** DISCREPANCIA: monitor={node.lap_count} vs '
                      f'nodo PID={len(pid_laps)} — investigar ***')
        else:
            print('  (sin líneas de vuelta encontradas)')

    if node.n_samples == 0:
        print('\n*** ADVERTENCIA: no se recibió ningún mensaje de IPS. '
              'Verifica que el simulador/bridge estén conectados. ***')

    if args.log_csv:
        log_path = os.path.expanduser(args.log_csv)
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        write_header = not os.path.isfile(log_path)
        with open(log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['tag', 'duration_s', 'n_samples', 'collided',
                                  'collision_events', 'min_clearance_m',
                                  'warn_events', 'lap_count', 'lap_times',
                                  'pid_log_lap_lines'])
            writer.writerow([
                args.tag, f'{elapsed:.1f}', node.n_samples, node.collided,
                len(node.collision_events),
                f'{node.min_clearance:.3f}' if node.min_clearance != float('inf') else '',
                node.warn_events, node.lap_count,
                ';'.join(f'{t:.3f}' for t in node.lap_times),
                len(pid_laps),
            ])
        print(f'\nResumen agregado a {log_path}')


if __name__ == '__main__':
    main()
