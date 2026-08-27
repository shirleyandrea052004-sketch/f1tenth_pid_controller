#!/usr/bin/env python3
"""
pid_controller_node.py
-----------------------
Nodo ROS 2 que implementa un controlador PID de seguimiento de trayectoria
(path tracking) para el vehículo F1TENTH dentro del simulador AutoDRIVE.
"""

import csv
import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import TransformException


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PID:
    def __init__(self, kp, ki, kd, integral_limit=1.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_error_valid = False

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_error_valid = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        p_term = self.kp * error

        tentative_integral = self._integral + error * dt
        tentative_integral = max(-self.integral_limit, min(self.integral_limit, tentative_integral))
        i_term = self.ki * tentative_integral

        if self._prev_error_valid:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        if self.output_limit is not None:
            clamped = max(-self.output_limit, min(self.output_limit, output))
            saturated = clamped != output
            # Anti-windup: si ya estamos saturados Y el error sigue empujando
            # en la misma dirección, NO confirmes el integral tentativo (evita
            # que siga creciendo mientras el output no puede hacer nada más,
            # lo cual causaría overshoot al liberarse la saturación).
            if not (saturated and (error > 0) == (output > 0)):
                self._integral = tentative_integral
            output = clamped
        else:
            self._integral = tentative_integral

        self._prev_error = error
        self._prev_error_valid = True
        return output


class PIDControllerNode(Node):

    def __init__(self):
        super().__init__('pid_controller_node')

        self.declare_parameter('path_source', 'csv')
        self.declare_parameter('path_csv', os.path.expanduser('~/f1tenth_waypoints/smoothed_path.csv'))
        self.declare_parameter('path_topic', '/smoothed_path')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('vehicle_frame', 'f1tenth_1')

        self.declare_parameter('steering_topic', '/autodrive/f1tenth_1/steering_command')
        self.declare_parameter('throttle_topic', '/autodrive/f1tenth_1/throttle_command')

        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('max_tf_age_sec', 1.0)

        # Descriptores para rqt_reconfigure
        kp_desc = ParameterDescriptor(
            description='Ganancia Proporcional',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.05)]
        )
        self.declare_parameter('kp_steer', 0.85, kp_desc)
        self.declare_parameter('ki_steer', 0.00)

        kd_desc = ParameterDescriptor(
            description='Ganancia Derivativa (Amortiguador)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)]
        )
        self.declare_parameter('kd_steer', 0.15, kd_desc)

        k_cte_desc = ParameterDescriptor(
            description='Fuerza de correccion lateral',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=3.0, step=0.1)]
        )
        self.declare_parameter('k_cte', 0.8, k_cte_desc)

        k_cte_curve_desc = ParameterDescriptor(
            description='Refuerzo de k_cte proporcional a la curvatura local (0 = desactivado)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)]
        )
        self.declare_parameter('k_cte_curve_gain', 0.0, k_cte_curve_desc)
        self.declare_parameter('heading_weight_speed_ref', 0.0)
        self.declare_parameter('heading_weight_min', 0.3)

        eps_desc = ParameterDescriptor(
            description='Suavizado de arranque',
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)]
        )
        self.declare_parameter('speed_epsilon', 1.2, eps_desc)

        self.declare_parameter('max_steering_rate', 3.0)
        self.declare_parameter('max_throttle_rate', 1.0)
        self.declare_parameter('cross_track_sign', 1.0)
        self.declare_parameter('steering_sign', -1.0)

        # Desactivado por defecto para liberar la terminal
        self.declare_parameter('debug_logging', False)
        self.declare_parameter('debug_period', 1.0)

        self.declare_parameter('kp_speed', 0.60)
        self.declare_parameter('ki_speed', 0.05)
        self.declare_parameter('kd_speed', 0.05)

        self.declare_parameter('max_steering_angle_rad', 0.5236)
        self.declare_parameter('max_throttle', 1.0)
        self.declare_parameter('min_throttle', 0.0)

        # Ley de dirección: Pure Pursuit apunta a un punto más adelante en
        # la ruta en vez de mezclar heading_error + término lateral (Stanley).
        # Es mucho más robusto en curvas cerradas a baja velocidad, porque
        # no depende de que heading y cross-track "coincidan en signo" —
        # siempre gira hacia donde está el punto objetivo, sin importar la
        # orientación instantánea del auto.
        self.declare_parameter('use_pure_pursuit', True)
        self.declare_parameter('wheelbase_m', 0.33)
        self.declare_parameter('min_lookahead_m', 0.6)
        self.declare_parameter('lookahead_speed_gain', 0.8)

        self.declare_parameter('base_speed', 1.5)
        self.declare_parameter('min_speed', 0.6)
        self.declare_parameter('curvature_gain', 6.0)
        self.declare_parameter('lookahead_curvature_pts', 8)
        self.declare_parameter('speed_lookahead_m', 0.0)

        self.declare_parameter('lap_trigger_radius', 0.45)  
        self.declare_parameter('lap_exit_radius_factor', 4.0)
        self.declare_parameter('min_lap_time', 8.0)  

        self.declare_parameter('stall_speed_threshold', 0.05)  
        self.declare_parameter('max_plausible_speed', 6.0)  # m/s
        self.declare_parameter('stall_timeout', 2.0)  

        # Lectura de parámetros
        self.path_source = self.get_parameter('path_source').value
        self.path_csv = self.get_parameter('path_csv').value
        self.path_topic = self.get_parameter('path_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.vehicle_frame = self.get_parameter('vehicle_frame').value

        steering_topic = self.get_parameter('steering_topic').value
        throttle_topic = self.get_parameter('throttle_topic').value

        self.control_period = 1.0 / float(self.get_parameter('control_frequency').value)
        self.max_tf_age_sec = float(self.get_parameter('max_tf_age_sec').value)

        self.max_steering_angle_rad = float(self.get_parameter('max_steering_angle_rad').value)
        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.min_throttle = float(self.get_parameter('min_throttle').value)

        self.use_pure_pursuit = bool(self.get_parameter('use_pure_pursuit').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.min_lookahead_m = float(self.get_parameter('min_lookahead_m').value)
        self.lookahead_speed_gain = float(self.get_parameter('lookahead_speed_gain').value)

        self.k_cte = float(self.get_parameter('k_cte').value)
        self.k_cte_curve_gain = float(self.get_parameter('k_cte_curve_gain').value)
        self.heading_weight_speed_ref = float(self.get_parameter('heading_weight_speed_ref').value)
        self.heading_weight_min = float(self.get_parameter('heading_weight_min').value)
        self.speed_epsilon = float(self.get_parameter('speed_epsilon').value)
        self.max_steering_rate = float(self.get_parameter('max_steering_rate').value)
        self.max_throttle_rate = float(self.get_parameter('max_throttle_rate').value)
        self._prev_steering_norm = 0.0
        self._prev_throttle = 0.0

        self.cross_track_sign = float(self.get_parameter('cross_track_sign').value)
        self.steering_sign = float(self.get_parameter('steering_sign').value)

        self.debug_logging = bool(self.get_parameter('debug_logging').value)
        self.debug_period = float(self.get_parameter('debug_period').value)
        self._last_debug_log = 0.0

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.curvature_gain = float(self.get_parameter('curvature_gain').value)
        self.lookahead_curvature_pts = int(self.get_parameter('lookahead_curvature_pts').value)
        self.speed_lookahead_m = float(self.get_parameter('speed_lookahead_m').value)

        self.lap_trigger_radius = float(self.get_parameter('lap_trigger_radius').value)
        self.lap_exit_radius = self.lap_trigger_radius * float(self.get_parameter('lap_exit_radius_factor').value)
        self.min_lap_time = float(self.get_parameter('min_lap_time').value)

        self.stall_speed_threshold = float(self.get_parameter('stall_speed_threshold').value)
        self.max_plausible_speed = float(self.get_parameter('max_plausible_speed').value)
        self.stall_timeout = float(self.get_parameter('stall_timeout').value)
        self._stall_since = None

        # PIDs
        self.steer_pid = PID(
            kp=float(self.get_parameter('kp_steer').value),
            ki=float(self.get_parameter('ki_steer').value),
            kd=float(self.get_parameter('kd_steer').value),
            integral_limit=1.0,
            output_limit=self.max_steering_angle_rad,
        )
        self.speed_pid = PID(
            kp=float(self.get_parameter('kp_speed').value),
            ki=float(self.get_parameter('ki_speed').value),
            kd=float(self.get_parameter('kd_speed').value),
            integral_limit=2.0,
            output_limit=self.max_throttle,
        )

        self.path_xy = []      
        self.path_yaw = []     
        self.path_ready = False
        self._nearest_idx_hint = 0

        self._last_pos = None
        self._last_real_update_time = None
        self._last_time = None
        self._speed_estimate = 0.0
        self._speed_history = deque(maxlen=5)

        self.lap_count = 0
        self.away_from_start = False
        self.race_start_time = None
        self.lap_start_time = None
        self.best_lap_time = None

        self.steering_pub = self.create_publisher(Float32, steering_topic, 10)
        self.throttle_pub = self.create_publisher(Float32, throttle_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.path_source == 'csv':
            self._load_path_from_csv(self.path_csv)
        else:
            qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(Path, self.path_topic, self._path_callback, qos)

        self.control_timer = self.create_timer(self.control_period, self._control_loop)
        self.add_on_set_parameters_callback(self._parameter_callback)

    def _parameter_callback(self, params):
        for param in params:
            name = param.name
            value = param.value

            # --- Steering PID ---
            if name == 'kp_steer':
                self.steer_pid.kp = value
            elif name == 'ki_steer':
                self.steer_pid.ki = value
            elif name == 'kd_steer':
                self.steer_pid.kd = value

            # --- Speed PID (antes no se sincronizaban) ---
            elif name == 'kp_speed':
                self.speed_pid.kp = value
            elif name == 'ki_speed':
                self.speed_pid.ki = value
            elif name == 'kd_speed':
                self.speed_pid.kd = value

            # --- Control lateral ---
            elif name == 'k_cte':
                self.k_cte = value
            elif name == 'k_cte_curve_gain':
                self.k_cte_curve_gain = value
            elif name == 'heading_weight_speed_ref':
                self.heading_weight_speed_ref = value
            elif name == 'heading_weight_min':
                self.heading_weight_min = value
            elif name == 'speed_epsilon':
                self.speed_epsilon = value
            elif name == 'cross_track_sign':
                self.cross_track_sign = value
            elif name == 'steering_sign':
                self.steering_sign = value
            elif name == 'max_steering_rate':
                self.max_steering_rate = value

            # --- Velocidad objetivo / curvatura (antes no se sincronizaban) ---
            elif name == 'base_speed':
                self.base_speed = value
            elif name == 'min_speed':
                self.min_speed = value
            elif name == 'curvature_gain':
                self.curvature_gain = value
            elif name == 'min_throttle':
                self.min_throttle = value
            elif name == 'max_throttle':
                self.max_throttle = value
            elif name == 'max_steering_angle_rad':
                self.max_steering_angle_rad = value
                self.steer_pid.output_limit = value
            elif name == 'use_pure_pursuit':
                self.use_pure_pursuit = bool(value)
            elif name == 'wheelbase_m':
                self.wheelbase_m = value
            elif name == 'min_lookahead_m':
                self.min_lookahead_m = value
            elif name == 'lookahead_speed_gain':
                self.lookahead_speed_gain = value
            elif name == 'lookahead_curvature_pts':
                self.lookahead_curvature_pts = int(value)
            elif name == 'speed_lookahead_m':
                self.speed_lookahead_m = value
            elif name == 'max_throttle_rate':
                self.max_throttle_rate = value

            # --- Debug ---
            elif name == 'debug_logging':
                self.debug_logging = value
            elif name == 'debug_period':
                self.debug_period = value

        return SetParametersResult(successful=True)

    def _load_path_from_csv(self, csv_path: str):
        if not os.path.isfile(csv_path):
            return
        xy = []
        yaw = []
        with open(csv_path, 'r', newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                try:
                    x = float(row[1])
                    y = float(row[2])
                    yw = float(row[3]) if len(row) > 3 else 0.0
                except (ValueError, IndexError):
                    continue
                xy.append((x, y))
                yaw.append(yw)

        if len(xy) < 2:
            return

        if all(y == 0.0 for y in yaw):
            yaw = self._compute_yaw_from_points(xy)

        self.path_xy = xy
        self.path_yaw = yaw
        self.path_ready = True

    def _path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        xy = []
        yaw = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            xy.append((p.x, p.y))
            yaw.append(yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.path_xy = xy
        self.path_yaw = yaw
        self.path_ready = True

    @staticmethod
    def _compute_yaw_from_points(xy):
        n = len(xy)
        yaw = [0.0] * n
        for i in range(n):
            j = min(i + 1, n - 1)
            dx = xy[j][0] - xy[i][0]
            dy = xy[j][1] - xy[i][1]
            yaw[i] = math.atan2(dy, dx)
        return yaw

    def _find_nearest_index(self, x, y):
        n = len(self.path_xy)
        window = 40  
        start = max(0, self._nearest_idx_hint - window)
        end = min(n, self._nearest_idx_hint + window)

        best_idx = start
        best_dist = float('inf')
        for i in range(start, end):
            px, py = self.path_xy[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_dist:
                best_dist = d
                best_idx = i

        if best_idx in (start, end - 1):
            for i in range(n):
                px, py = self.path_xy[i]
                d = (px - x) ** 2 + (py - y) ** 2
                if d < best_dist:
                    best_dist = d
                    best_idx = i

        self._nearest_idx_hint = best_idx
        return best_idx, math.sqrt(best_dist)

    def _find_lookahead_point(self, idx, lookahead_dist):
        """Primer punto de la ruta, buscando hacia adelante desde `idx`,
        cuya distancia en línea recta a `idx` sea >= lookahead_dist. Si la
        ruta se acaba antes, devuelve el último punto."""
        n = len(self.path_xy)
        x0, y0 = self.path_xy[idx]
        j = idx
        while j < n - 1:
            xj, yj = self.path_xy[j]
            if math.hypot(xj - x0, yj - y0) >= lookahead_dist:
                return j
            j += 1
        return n - 1

    def _signed_cross_track_error(self, x, y, idx):
        px, py = self.path_xy[idx]
        path_yaw = self.path_yaw[idx]
        dx = x - px
        dy = y - py
        return -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

    def _estimate_curvature(self, idx):
        n = len(self.path_xy)
        j = min(idx + self.lookahead_curvature_pts, n - 1)
        if j == idx:
            return 0.0
        dyaw = normalize_angle(self.path_yaw[j] - self.path_yaw[idx])
        px, py = self.path_xy[idx]
        qx, qy = self.path_xy[j]
        ds = math.hypot(qx - px, qy - py)
        if ds < 1e-3:
            return 0.0
        return abs(dyaw) / ds

    def _max_curvature_ahead(self, idx, lookahead_m):
        """Curvatura máxima entre `idx` y `idx` + lookahead_m (en metros),
        muestreada con la misma ventana local que `_estimate_curvature`.
        A diferencia de usar un único par de puntos muy separados (que se
        vuelve inestable si el tramo invierte de dirección, como en una
        horquilla cerrada), esto detecta con antelación el tramo más
        cerrado que se viene, para frenar ANTES de llegar a él en vez de
        reaccionar solo cuando ya se está en el punto más cerrado."""
        n = len(self.path_xy)
        max_kappa = 0.0
        j = idx
        traveled = 0.0
        while j < n - 1 and traveled < lookahead_m:
            kappa = self._estimate_curvature(j)
            if kappa > max_kappa:
                max_kappa = kappa
            px, py = self.path_xy[j]
            qx, qy = self.path_xy[j + 1]
            traveled += math.hypot(qx - px, qy - py)
            j += 1
        return max_kappa

    def _publish_zero_commands(self):
        self.steering_pub.publish(Float32(data=0.0))
        self.throttle_pub.publish(Float32(data=0.0))
        self._prev_steering_norm = 0.0
        self._prev_throttle = 0.0

    def _control_loop(self):
        if not self.path_ready or len(self.path_xy) < 2:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.vehicle_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        tf_age = self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)
        if tf_age.nanoseconds * 1e-9 > self.max_tf_age_sec:
            # La transformada map->vehicle no se ha actualizado en un
            # buen rato (p. ej. el bridge/sim se desconectó o el TF
            # broadcaster se quedó republicando datos viejos). Seguir
            # publicando comandos sobre una pose obsoleta sería conducir
            # a ciegas, así que se corta throttle/steering y se espera.
            self._publish_zero_commands()
            if self.debug_logging and (
                self.get_clock().now().nanoseconds * 1e-9 - self._last_debug_log
            ) >= self.debug_period:
                self._last_debug_log = self.get_clock().now().nanoseconds * 1e-9
                self.get_logger().warn(
                    f'TF de {self.vehicle_frame} obsoleta '
                    f'({tf_age.nanoseconds * 1e-9:.2f}s) — deteniendo por seguridad.')
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        # Umbral para distinguir una posición nueva de una repetida (el TF se
        # republica a 20Hz, pero la fuente real /ips solo actualiza a ~1.4Hz;
        # sin este filtro, la mayoría de ciclos ven dist=0 y arrastran
        # _speed_estimate artificialmente hacia 0, apagando el steering).
        min_move_eps = 0.01  # metros

        # Si de verdad han pasado >1s sin una posición nueva, el auto está
        # parado (no es solo que /ips todavía no manda su siguiente muestra
        # a ~1.4Hz) -> hay que dejar que _speed_estimate baje a 0 en vez de
        # quedar congelada en el último valor (lo cual causaría un stall
        # permanente: el controlador "cree" que sigue rápido y no da throttle).
        STALE_TIMEOUT = 1.0  # segundos

        if self._last_pos is not None and self._last_time is not None:
            dist = math.hypot(x - self._last_pos[0], y - self._last_pos[1])
            if dist > min_move_eps:
                dt_speed = now_sec - self._last_time
                raw_speed = dist / dt_speed if dt_speed > 1e-4 else None
                is_teleport = (dist > 2.0) or (
                    raw_speed is not None and raw_speed > self.max_plausible_speed)
                if is_teleport:
                    # Detección de teletransporte/reset (salto de posición O
                    # velocidad físicamente imposible, ej. un reset manual en
                    # Unity que el cálculo dist/dt interpreta como movimiento
                    # real). Sin esto, un reset produce un v_est falso y
                    # gigante, lo que dispara errores de heading enormes y
                    # el steering se satura -> choque inmediato.
                    self._speed_history.clear()
                    self._speed_estimate = 0.0
                    self.steer_pid.reset()
                    self.speed_pid.reset()
                    self._nearest_idx_hint = 0  # forzar búsqueda completa
                elif raw_speed is not None:
                    self._speed_history.append(raw_speed)
                    self._speed_estimate = sum(self._speed_history) / len(self._speed_history)
                # Solo avanzamos la referencia cuando hubo una posición
                # realmente nueva, para que el próximo dt_speed sea correcto.
                self._last_pos = (x, y)
                self._last_time = now_sec
                self._last_real_update_time = now_sec
            elif (self._last_real_update_time is not None and
                    (now_sec - self._last_real_update_time) > STALE_TIMEOUT):
                self._speed_history.append(0.0)
                self._speed_estimate = sum(self._speed_history) / len(self._speed_history)
        else:
            self._last_pos = (x, y)
            self._last_time = now_sec
            self._last_real_update_time = now_sec

        if self._speed_estimate < self.stall_speed_threshold:
            if self._stall_since is None:
                self._stall_since = now_sec
            elif (now_sec - self._stall_since) > self.stall_timeout:
                self.steer_pid.reset()
                self.speed_pid.reset()
                self._stall_since = now_sec  
        else:
            self._stall_since = None

        idx, _ = self._find_nearest_index(x, y)
        path_yaw = self.path_yaw[idx]

        cross_track_error = self.cross_track_sign * self._signed_cross_track_error(x, y, idx)
        heading_error = normalize_angle(path_yaw - yaw)

        # k_cte efectivo: se refuerza en curvas cerradas (curvatura local alta)
        # para dar más autoridad de corrección ahí, sin desestabilizar los
        # tramos rectos (donde curvature ~ 0 y k_cte se queda en su base).
        curvature = self._estimate_curvature(idx)
        effective_k_cte = self.k_cte + self.k_cte_curve_gain * curvature

        dt = self.control_period

        if self.use_pure_pursuit:
            # Pure Pursuit: apunta a un punto más adelante en la ruta en vez
            # de mezclar heading_error + término lateral. El ángulo de
            # dirección sale directo de la geometría triángulo-auto→punto,
            # así que no depende de que ambos términos "coincidan en signo"
            # como en Stanley — es robusto incluso en horquillas donde el
            # heading del auto todavía no se pareja con el de la ruta.
            lookahead_dist = max(self.min_lookahead_m,
                                  self.lookahead_speed_gain * self._speed_estimate)
            target_idx = self._find_lookahead_point(idx, lookahead_dist)
            tx, ty = self.path_xy[target_idx]
            dx_l = tx - x
            dy_l = ty - y
            local_x = math.cos(yaw) * dx_l + math.sin(yaw) * dy_l
            local_y = -math.sin(yaw) * dx_l + math.cos(yaw) * dy_l
            Ld = math.hypot(local_x, local_y)
            if Ld < 1e-3:
                steering_angle = 0.0
            else:
                alpha = math.atan2(local_y, local_x)
                steering_angle = math.atan2(2.0 * self.wheelbase_m * math.sin(alpha), Ld)
            steering_angle = max(-self.max_steering_angle_rad,
                                  min(self.max_steering_angle_rad, steering_angle))
        else:
            lateral_term = math.atan2(effective_k_cte * cross_track_error,
                                       self._speed_estimate + self.speed_epsilon)

            # A velocidad casi nula, heading_error y lateral_term a veces
            # quedan con signos opuestos (el auto "ya casi" apunta hacia
            # adelante pero sigue muy lejos del centro de la ruta) y se
            # cancelan, dejando un combined_error pequeño que no alcanza
            # para sacar al auto de un error lateral grande. El heading
            # solo importa una vez que el auto realmente avanza (define
            # hacia dónde apunta el próximo tramo); detenido, lo único
            # accionable es apuntar las ruedas hacia la ruta. Por eso el
            # peso de heading_error se reduce a velocidades bajas y solo
            # recupera peso completo una vez en movimiento.
            heading_weight = 1.0
            if self.heading_weight_speed_ref > 0.0:
                heading_weight = max(self.heading_weight_min,
                                      min(1.0, self._speed_estimate / self.heading_weight_speed_ref))

            combined_error = heading_weight * heading_error + lateral_term
            steering_angle = self.steer_pid.update(combined_error, dt)

        steering_norm_raw = self.steering_sign * (steering_angle / self.max_steering_angle_rad)
        steering_norm_raw = max(-1.0, min(1.0, steering_norm_raw))

        max_delta = self.max_steering_rate * dt
        steering_norm = self._prev_steering_norm + max(
            -max_delta, min(max_delta, steering_norm_raw - self._prev_steering_norm)
        )
        self._prev_steering_norm = steering_norm

        # Frenar en anticipación: usar la curvatura más cerrada que aparezca
        # dentro de speed_lookahead_m metros hacia adelante (no solo la
        # curvatura puntual en idx), para reducir velocidad ANTES de llegar
        # a una horquilla cerrada en vez de reaccionar ya sobre ella.
        braking_curvature = curvature
        if self.speed_lookahead_m > 0.0:
            braking_curvature = max(curvature, self._max_curvature_ahead(idx, self.speed_lookahead_m))

        target_speed = self.base_speed / (1.0 + self.curvature_gain * braking_curvature)
        target_speed = max(self.min_speed, min(self.base_speed, target_speed))

        speed_error = target_speed - self._speed_estimate
        throttle_raw = self.speed_pid.update(speed_error, dt)
        throttle_raw = max(self.min_throttle, min(self.max_throttle, throttle_raw))

        max_dthrottle = self.max_throttle_rate * dt
        throttle = self._prev_throttle + max(
            -max_dthrottle, min(max_dthrottle, throttle_raw - self._prev_throttle)
        )
        throttle = max(self.min_throttle, min(self.max_throttle, throttle))
        self._prev_throttle = throttle

        steer_msg = Float32()
        steer_msg.data = float(steering_norm)
        self.steering_pub.publish(steer_msg)

        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)

        # Log opcional solo si se activa explícitamente en rqt
        if self.debug_logging and (now_sec - self._last_debug_log) >= self.debug_period:
            self._last_debug_log = now_sec
            self.get_logger().info(
                f'idx={idx:4d} | e_y={cross_track_error:+.3f} m | e_head={heading_error:+.3f} rad | '
                f'steer_cmd={steering_norm:+.3f} | v_est={self._speed_estimate:.2f} | '
                f'v_target={target_speed:.2f} | curv={curvature:.2f} | throttle={throttle:.3f} | '
                f'stall_since={self._stall_since}'
            )

        self._update_lap_tracking(x, y, now)

    def _update_lap_tracking(self, x, y, now):
        start_x, start_y = self.path_xy[0]
        dist_to_start = math.hypot(x - start_x, y - start_y)

        if self.race_start_time is None:
            self.race_start_time = now
            self.lap_start_time = now

        if not self.away_from_start:
            if dist_to_start > self.lap_exit_radius:
                self.away_from_start = True
        else:
            if dist_to_start < self.lap_trigger_radius:
                elapsed = (now.nanoseconds - self.lap_start_time.nanoseconds) * 1e-9
                if elapsed > self.min_lap_time:
                    self.lap_count += 1
                    self.lap_start_time = now
                    self.away_from_start = False


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()