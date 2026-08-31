#!/usr/bin/env python3
"""
pid_controller_node.py
-----------------------
Nodo ROS 2 que implementa un controlador de seguimiento de trayectoria
(path tracking) para el vehículo F1TENTH dentro del simulador AutoDRIVE.

Ley de dirección: Pure Pursuit (geométrico) con limitador de slew rate.
Ley de velocidad: feedforward + PID correctivo sobre la velocidad estimada.

NOTAS DE LA REVISIÓN (basada en telemetría real de pista)
---------------------------------------------------------
El problema que impedía completar una vuelta NO era el ajuste de ganancias,
sino un lazo de realimentación roto en la estimación de velocidad:

  1. El detector de teletransporte usaba un umbral FIJO de 2.0 m. Como /ips
     publica a ~1.4 Hz (0.7 s entre muestras), un auto a 3 m/s recorre 2.1 m
     entre muestras -> se clasificaba como "teletransporte" -> se forzaba
     v_est = 0.0. Con v_est=0 el PID de velocidad veía un error enorme y
     saturaba el throttle en 1.0, lo que aceleraba más el auto, lo que
     agrandaba los saltos, lo que producía más falsos teletransportes...
     Círculo vicioso: el auto llegó a 4.3 m/s con v_target=1.85 m/s.
     -> Corregido: el umbral ahora escala con el tiempo transcurrido.

  2. El lazo corría a 20 Hz pero la pose sólo se actualiza a ~1.4 Hz, así
     que el PID integraba ~14 veces el MISMO error viejo (windup puro).
     Por eso el throttle subía 0.05 -> 0.60 -> 1.00 con el auto detenido.
     -> Corregido: el PID sólo se actualiza cuando llega una pose nueva.

  3. Sin feedforward, todo el esfuerzo de throttle salía del término
     integral, que es justamente el más lento y el que más windup sufre.
     -> Corregido: throttle_feedforward_per_mps aporta el punto de
        operación y el PID sólo corrige la diferencia.

  4. Tras un corte de TF el auto sigue moviéndose "a ciegas" y al volver
     la pose el heading podía diferir >1 rad, provocando un latigazo de
     dirección a máxima autoridad.
     -> Corregido: recuperación suave (reset de PIDs + rebúsqueda de índice).
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

        # 10 Hz en vez de 20: con /ips a ~1.4 Hz, correr a 20 Hz sólo
        # re-procesa datos viejos y consume CPU que agrava el lag del sim.
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_tf_age_sec', 1.5)

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

        # Bajado de 3.0 -> 2.0: limita el latigazo de dirección cuando la
        # pose vuelve después de un corte de TF con heading muy divergente.
        self.declare_parameter('max_steering_rate', 2.0)
        self.declare_parameter('max_throttle_rate', 0.5)
        self.declare_parameter('cross_track_sign', 1.0)
        self.declare_parameter('steering_sign', -1.0)

        # Telemetria por ciclo (idx, e_y, v_est, thr...). Util para depurar,
        # pero satura la consola en carrera: con debug_logging=false solo se
        # imprimen las vueltas completadas y los avisos de seguridad.
        self.declare_parameter('debug_logging', False)
        self.declare_parameter('debug_period', 1.0)

        # Ganancias de velocidad recalibradas: con feedforward activo, el PID
        # ya no tiene que generar el punto de operación completo, sólo
        # corregir la diferencia -> ganancias mucho más suaves y ki mínimo
        # (el windup del integral era la causa del throttle clavado en 1.0).
        self.declare_parameter('kp_speed', 0.25)
        self.declare_parameter('ki_speed', 0.02)
        self.declare_parameter('kd_speed', 0.02)

        # Feedforward de throttle: cuánto throttle hace falta por cada m/s
        # deseado. El valor inicial (0.23) venía de medir throttle=1.0 -> 4.3
        # m/s durante una corrida con el estimador de velocidad roto, así que
        # estaba inflado. Con telemetría limpia: thr~0.22 sostenía ~1.4 m/s
        # en recta, o sea ~0.157. Con 0.23 el auto iba ~50% por encima del
        # objetivo en las rectas y llegaba pasado de vueltas a la curva
        # siguiente, que es lo que producía el serpenteo.
        self.declare_parameter('throttle_feedforward_per_mps', 0.16)

        self.declare_parameter('max_steering_angle_rad', 0.5236)
        # TECHO DE SEGURIDAD. Aunque la estimación de velocidad vuelva a
        # fallar, el auto no puede superar ~1.9 m/s con este límite, lo que
        # deja margen para recuperar el control en vez de salir de pista.
        self.declare_parameter('max_throttle', 0.45)
        self.declare_parameter('min_throttle', 0.0)

        # Ley de dirección: Pure Pursuit apunta a un punto más adelante en
        # la ruta en vez de mezclar heading_error + término lateral (Stanley).
        # Es mucho más robusto en curvas cerradas a baja velocidad, porque
        # no depende de que heading y cross-track "coincidan en signo" —
        # siempre gira hacia donde está el punto objetivo, sin importar la
        # orientación instantánea del auto.
        self.declare_parameter('use_pure_pursuit', True)
        self.declare_parameter('wheelbase_m', 0.33)
        # Lookahead subido de 0.7 -> 1.3 m. Con /ips a ~1.4Hz el retardo de
        # la pose (~0.7s) hace que a 1.3 m/s el auto avance ~0.9 m entre
        # actualizaciones: un lookahead de 0.7 m era MENOR que el propio
        # retardo, así que el auto apuntaba a un punto que ya había pasado
        # -> sobreviraje y oscilación. Un lookahead más largo suaviza la
        # respuesta y da margen al retardo.
        self.declare_parameter('min_lookahead_m', 1.3)
        self.declare_parameter('lookahead_speed_gain', 1.0)

        # Compensación de latencia (dead reckoning). En vez de controlar
        # sobre la última pose conocida (hasta 0.7s vieja), se proyecta la
        # posición y el rumbo hacia adelante usando el modelo de bicicleta
        # con la velocidad estimada y el último comando de dirección. Esto
        # ataca la causa raíz del retardo en vez de solo suavizar la
        # respuesta.
        self.declare_parameter('latency_compensation', True)
        self.declare_parameter('max_prediction_sec', 0.8)

        # Ventana del promedio de velocidad. Con /ips a ~1.4Hz, 5 muestras
        # abarcaban ~3.5s de historia: v_est quedaba muy por detrás de la
        # velocidad real (reportaba 0.43 m/s cuando el auto iba a 1.36 m/s),
        # y como el lookahead se calcula a partir de v_est, eso producía un
        # lookahead demasiado corto justo cuando más falta hacía.
        self.declare_parameter('speed_history_len', 8)
        # Ventana temporal (s) sobre la que se promedia la velocidad. Debe
        # cubrir varias muestras de /ips (~1.4 Hz) para que la irregularidad
        # de llegada no se traduzca en ruido de velocidad.
        self.declare_parameter('speed_window_sec', 1.5)

        # ---- Cierre de la trayectoria en bucle continuo ----
        # La ruta generada por Dijkstra es un arreglo ABIERTO: termina en el
        # goal, que está al otro lado de la pared virtual y a ~1.3 m del
        # punto de inicio, con un salto de rumbo de ~52°. Al llegar al final
        # el controlador se quedaba sin puntos hacia adelante y perdía el
        # control. Aquí la ruta se cierra sobre sí misma:
        #   1. Se recortan los últimos puntos que se desvían hacia el goal
        #      (son un artefacto de dónde quedó el goal, no parte de la pista).
        #   2. Se inserta un puente Hermite que respeta las tangentes de
        #      ambos extremos, remuestreado al mismo espaciado que la ruta.
        #   3. Todos los índices pasan a circular con módulo n.
        self.declare_parameter('loop_path', True)
        # Modo de empalme entre el final de la ruta y el inicio:
        #   "straight" -> línea recta. En la pista real no existe la pared
        #                 virtual (solo se dibujó en la imagen del mapa para
        #                 forzar a Dijkstra a rodear el circuito), la pista
        #                 es ancha y el auto gira lo suficiente, así que la
        #                 unión directa es válida y más predecible.
        #   "hermite"  -> spline cúbico que respeta las tangentes de ambos
        #                 extremos. Empalme sin quiebres de rumbo, útil si
        #                 el hueco es grande o los rumbos muy distintos.
        self.declare_parameter('loop_bridge_mode', 'straight')
        # Máximo de puntos de cola que se pueden recortar buscando un empalme
        # físicamente realizable.
        self.declare_parameter('loop_max_trim', 12)
        # Margen de seguridad sobre el radio de giro mínimo del vehículo.
        self.declare_parameter('loop_curvature_margin', 0.8)
        # Quiebre de rumbo máximo tolerado en las uniones del empalme.
        # 0.44 rad = 25 grados: por encima de eso el vehículo llega al
        # empalme con el morro claramente desalineado y corrige a tirones
        # (zigzagueo al cerrar la vuelta).
        self.declare_parameter('loop_max_kink_rad', 0.44)

        # Perfil de velocidad conservador para la Prueba 1 (10 vueltas sin
        # colisión). Para la Prueba 2 (vuelta rápida) subir base_speed
        # gradualmente y verificar que v_est siga a la velocidad real.
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('curvature_gain', 9.0)
        self.declare_parameter('lookahead_curvature_pts', 10)
        self.declare_parameter('speed_lookahead_m', 1.2)

        self.declare_parameter('lap_trigger_radius', 0.45)
        self.declare_parameter('lap_exit_radius_factor', 4.0)
        self.declare_parameter('min_lap_time', 8.0)

        self.declare_parameter('stall_speed_threshold', 0.05)
        self.declare_parameter('max_plausible_speed', 6.0)  # m/s
        self.declare_parameter('stall_timeout', 3.0)

        # Cuánto tiempo puede pasar desde la última pose REAL para que el
        # ciclo actual se considere "fresco" y pueda alimentar los PIDs.
        # Debe ser del orden del período del lazo, no del de /ips.
        self.declare_parameter('pose_fresh_window_sec', 0.05)

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
        self.throttle_ff_per_mps = float(self.get_parameter('throttle_feedforward_per_mps').value)

        self.use_pure_pursuit = bool(self.get_parameter('use_pure_pursuit').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.min_lookahead_m = float(self.get_parameter('min_lookahead_m').value)
        self.lookahead_speed_gain = float(self.get_parameter('lookahead_speed_gain').value)
        self.latency_compensation = bool(self.get_parameter('latency_compensation').value)
        self.max_prediction_sec = float(self.get_parameter('max_prediction_sec').value)
        self.speed_history_len = int(self.get_parameter('speed_history_len').value)
        self.speed_window_sec = float(self.get_parameter('speed_window_sec').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)
        self.loop_bridge_mode = str(self.get_parameter('loop_bridge_mode').value).lower()
        self.loop_max_trim = int(self.get_parameter('loop_max_trim').value)
        self.loop_curvature_margin = float(self.get_parameter('loop_curvature_margin').value)
        self.loop_max_kink_rad = float(self.get_parameter('loop_max_kink_rad').value)

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
        self._last_tf_error_log = 0.0

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
        self.pose_fresh_window = float(self.get_parameter('pose_fresh_window_sec').value)
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
            integral_limit=1.0,
            # El PID sólo aporta la CORRECCIÓN sobre el feedforward, así que
            # su salida se limita a una fracción del rango total.
            output_limit=self.max_throttle,
        )

        self.path_xy = []
        self.path_yaw = []
        self.path_ready = False
        self.path_is_loop = False
        self._nearest_idx_hint = 0

        self._last_pos = None
        self._last_real_update_time = None
        self._last_time = None
        self._speed_estimate = 0.0
        # Historial de poses (t, x, y) para el cálculo de velocidad.
        self._pose_history = deque(maxlen=max(2, self.speed_history_len))

        # Estado para el lazo de velocidad con datos intermitentes
        self._last_pid_correction = 0.0
        self._recovering_from_stale = False
        self._pose_is_fresh = False

        self.lap_count = 0
        self.away_from_start = False
        self.race_start_time = None
        self.lap_start_time = None
        self.best_lap_time = None
        self._lap_times = []

        self.steering_pub = self.create_publisher(Float32, steering_topic, 10)
        self.throttle_pub = self.create_publisher(Float32, throttle_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.path_source == 'csv':
            self._load_path_from_csv(self.path_csv)
            if self.path_ready and self.loop_path:
                self._close_path_loop()
            if self.path_ready:
                self.get_logger().info(
                    f'Trayectoria cargada desde CSV: {len(self.path_xy)} puntos '
                    f'({self.path_csv})')
            else:
                self.get_logger().error(
                    f'NO se pudo cargar la trayectoria desde {self.path_csv}. '
                    f'El auto no se moverá. Regenera el CSV con el planificador global.')
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

            # --- Speed PID ---
            elif name == 'kp_speed':
                self.speed_pid.kp = value
            elif name == 'ki_speed':
                self.speed_pid.ki = value
            elif name == 'kd_speed':
                self.speed_pid.kd = value
            elif name == 'throttle_feedforward_per_mps':
                self.throttle_ff_per_mps = value

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

            # --- Velocidad objetivo / curvatura ---
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
                self.speed_pid.output_limit = value
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
            elif name == 'latency_compensation':
                self.latency_compensation = bool(value)
            elif name == 'max_prediction_sec':
                self.max_prediction_sec = value
            elif name == 'loop_max_kink_rad':
                self.loop_max_kink_rad = value
            elif name == 'loop_bridge_mode':
                self.loop_bridge_mode = str(value).lower()
            elif name == 'speed_history_len':
                self.speed_history_len = int(value)
                self._pose_history = deque(self._pose_history,
                                           maxlen=max(2, int(value)))
            elif name == 'speed_window_sec':
                self.speed_window_sec = value
            elif name == 'lookahead_curvature_pts':
                self.lookahead_curvature_pts = int(value)
            elif name == 'speed_lookahead_m':
                self.speed_lookahead_m = value
            elif name == 'max_throttle_rate':
                self.max_throttle_rate = value
            elif name == 'max_tf_age_sec':
                self.max_tf_age_sec = value
            elif name == 'stall_timeout':
                self.stall_timeout = value
            elif name == 'pose_fresh_window_sec':
                self.pose_fresh_window = value

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
        self.path_is_loop = False
        if self.loop_path:
            self._close_path_loop()

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

    def _close_path_loop(self):
        """Convierte la ruta abierta (inicio -> goal) en un circuito cerrado.

        El planificador global entrega un arreglo que termina en el goal.
        Como el goal está al otro lado de la pared virtual, queda a ~1.3 m
        del punto de inicio y con un salto de rumbo grande: al llegar ahí el
        controlador se quedaba sin puntos hacia adelante y perdía el control.

        Los últimos puntos suelen desviarse para "alcanzar" el goal y no son
        parte de la línea de carrera; se prueban distintos recortes de cola y
        se elige el primero cuyo empalme sea físicamente realizable por el
        vehículo (curvatura por debajo de su radio de giro mínimo).
        """
        n = len(self.path_xy)
        if n < 5:
            return

        spacing = sum(
            math.hypot(self.path_xy[i + 1][0] - self.path_xy[i][0],
                       self.path_xy[i + 1][1] - self.path_xy[i][1])
            for i in range(n - 1)) / (n - 1)
        if spacing < 1e-6:
            return

        x1, y1 = self.path_xy[0]
        yaw1 = self.path_yaw[0]

        # Curvatura máxima que el vehículo puede seguir, con margen.
        r_min = self.wheelbase_m / max(1e-6, math.tan(self.max_steering_angle_rad))
        max_kappa = (1.0 / r_min) * self.loop_curvature_margin

        best = None
        fallback = None
        for trim in range(0, min(self.loop_max_trim, n - 3) + 1):
            i = n - 1 - trim
            x0, y0 = self.path_xy[i]
            yaw0 = self.path_yaw[i]
            gap = math.hypot(x1 - x0, y1 - y0)
            if gap < 1e-6:
                continue
            samples = self._bridge_samples(x0, y0, yaw0, x1, y1, yaw1, gap)
            kappa = self._max_discrete_curvature(samples)

            # La curvatura interna del puente NO basta como criterio: una
            # recta tiene curvatura 0 por definición, pero puede unirse a la
            # ruta con un quiebre de rumbo brusco en los extremos. Ese
            # quiebre es justamente lo que hace zigzaguear al vehículo al
            # cerrar la vuelta, así que se mide y se penaliza aquí.
            entry_kink = abs(normalize_angle(
                math.atan2(samples[1][1] - samples[0][1],
                           samples[1][0] - samples[0][0]) - yaw0))
            exit_kink = abs(normalize_angle(
                yaw1 - math.atan2(samples[-1][1] - samples[-2][1],
                                  samples[-1][0] - samples[-2][0])))
            max_kink = max(entry_kink, exit_kink)

            if kappa > max_kappa or max_kink > self.loop_max_kink_rad:
                if fallback is None or max_kink < fallback[6]:
                    fallback = (trim, i, samples, kappa, gap,
                                self._max_curvature_between(max(0, i - 30), i),
                                max_kink)
                continue

            # No basta con que el PUENTE sea seguible: hay que comprobar que
            # la cola que se CONSERVA también lo sea. El planificador puede
            # dejar un giro más cerrado que el radio mínimo del vehículo en
            # los últimos puntos (los que se curvan para alcanzar el goal),
            # y conservarlos significaría pedirle al auto algo que no puede
            # hacer, justo en la zona de meta.
            tail_kappa = self._max_curvature_between(max(0, i - 30), i)
            if tail_kappa > max_kappa:
                if fallback is None or max_kink < fallback[6]:
                    fallback = (trim, i, samples, kappa, gap, tail_kappa, max_kink)
                continue

            best = (trim, i, samples, kappa, gap, tail_kappa, max_kink)
            break

        if best is None and fallback is not None:
            # Ningún recorte deja una cola limpia dentro de loop_max_trim.
            # Se usa el mejor empalme disponible y se avisa, porque el
            # problema está en la ruta generada, no en el cierre.
            best = fallback
            self.get_logger().warn(
                f'Empalme no ideal: curvatura de cola {fallback[5]:.2f} 1/m '
                f'(límite {max_kappa:.2f}), quiebre de unión '
                f'{math.degrees(fallback[6]):.0f}° (límite '
                f'{math.degrees(self.loop_max_kink_rad):.0f}°). Se usa el mejor '
                f'disponible; considera loop_bridge_mode:="hermite".')

        if best is None:
            self.get_logger().warn(
                'No se encontró un empalme viable para cerrar el bucle; la ruta '
                'queda abierta. Revisa la posición del goal o sube loop_max_trim.')
            return

        trim, i, samples, kappa, gap, tail_kappa, max_kink = best

        # Remuestrear el puente al mismo espaciado que el resto de la ruta,
        # excluyendo los extremos (ya existen como path[i] y path[0]).
        bridge = self._resample_by_arclength(samples, spacing)

        new_xy = self.path_xy[:i + 1] + bridge
        new_yaw = self._compute_yaw_from_points(new_xy + [self.path_xy[0]])[:len(new_xy)]

        self.path_xy = new_xy
        self.path_yaw = new_yaw
        self.path_is_loop = True

        self.get_logger().info(
            f'Bucle cerrado: {trim} punto(s) de cola recortados, '
            f'{len(bridge)} punto(s) de empalme insertados sobre un hueco de '
            f'{gap:.2f} m en modo "{self.loop_bridge_mode}" '
            f'(curvatura empalme {kappa:.2f} 1/m, cola {tail_kappa:.2f} 1/m, '
            f'quiebre unión {math.degrees(max_kink):.0f}°). '
            f'Ruta final: {len(self.path_xy)} puntos.')

    def _max_curvature_between(self, i0, i1, window_m=0.5):
        """Curvatura máxima sostenida entre dos índices del arreglo actual.

        Se mide el cambio de rumbo sobre una ventana de ~window_m metros de
        arco en vez de entre puntos consecutivos. Un pico aislado de un solo
        punto (típico de un "kink" que deja el suavizador al forzar el paso
        exacto por el goal) no representa algo que el vehículo deba seguir:
        Pure Pursuit lo promedia dentro de su lookahead. Lo que sí importa
        es una curvatura sostenida a lo largo de varios decímetros, que es
        lo que este cálculo detecta.
        """
        max_kappa = 0.0
        i0 = max(0, i0)
        i1 = min(i1, len(self.path_xy) - 1)
        for i in range(i0, i1):
            traveled = 0.0
            j = i
            while j < i1 and traveled < window_m:
                traveled += math.hypot(self.path_xy[j + 1][0] - self.path_xy[j][0],
                                       self.path_xy[j + 1][1] - self.path_xy[j][1])
                j += 1
            if j == i or traveled < 1e-6:
                continue
            dtheta = normalize_angle(self.path_yaw[j] - self.path_yaw[i])
            max_kappa = max(max_kappa, abs(dtheta) / traveled)
        return max_kappa

    def _bridge_samples(self, x0, y0, yaw0, x1, y1, yaw1, scale, n_samples=200):
        """Genera el empalme según loop_bridge_mode.

        En modo "straight" el empalme es la unión directa. Los quiebres de
        rumbo en los extremos los absorbe Pure Pursuit dentro de su
        lookahead, así que no hace falta forzar continuidad de tangente si
        el hueco es corto y la pista ancha.
        """
        if self.loop_bridge_mode == 'straight':
            return [(x0 + (x1 - x0) * k / n_samples,
                     y0 + (y1 - y0) * k / n_samples)
                    for k in range(n_samples + 1)]
        return self._hermite_samples(x0, y0, yaw0, x1, y1, yaw1, scale, n_samples)

    @staticmethod
    def _hermite_samples(x0, y0, yaw0, x1, y1, yaw1, scale, n_samples=200):
        """Spline cúbico de Hermite entre dos poses, respetando las tangentes.
        Garantiza continuidad de rumbo en ambos empalmes (C1)."""
        mx0, my0 = math.cos(yaw0) * scale, math.sin(yaw0) * scale
        mx1, my1 = math.cos(yaw1) * scale, math.sin(yaw1) * scale
        out = []
        for k in range(n_samples + 1):
            t = k / n_samples
            t2, t3 = t * t, t * t * t
            h00 = 2 * t3 - 3 * t2 + 1
            h10 = t3 - 2 * t2 + t
            h01 = -2 * t3 + 3 * t2
            h11 = t3 - t2
            out.append((h00 * x0 + h10 * mx0 + h01 * x1 + h11 * mx1,
                        h00 * y0 + h10 * my0 + h01 * y1 + h11 * my1))
        return out

    @staticmethod
    def _max_discrete_curvature(samples):
        max_kappa = 0.0
        for k in range(1, len(samples) - 1):
            ax = samples[k][0] - samples[k - 1][0]
            ay = samples[k][1] - samples[k - 1][1]
            bx = samples[k + 1][0] - samples[k][0]
            by = samples[k + 1][1] - samples[k][1]
            ds = math.hypot(bx, by)
            if ds < 1e-9 or math.hypot(ax, ay) < 1e-9:
                continue
            dtheta = normalize_angle(math.atan2(by, bx) - math.atan2(ay, ax))
            max_kappa = max(max_kappa, abs(dtheta) / ds)
        return max_kappa

    @staticmethod
    def _resample_by_arclength(samples, spacing):
        """Remuestrea una polilínea densa a un espaciado uniforme,
        excluyendo el primer y el último punto (que ya existen en la ruta)."""
        out = []
        acc = 0.0
        for k in range(1, len(samples)):
            seg = math.hypot(samples[k][0] - samples[k - 1][0],
                             samples[k][1] - samples[k - 1][1])
            acc += seg
            if acc >= spacing:
                out.append(samples[k])
                acc = 0.0
        # El último punto del puente coincide con path[0]; se descarta para
        # no duplicarlo al cerrar el módulo.
        if out and math.hypot(out[-1][0] - samples[-1][0],
                              out[-1][1] - samples[-1][1]) < spacing * 0.5:
            out.pop()
        return out

    def _find_nearest_index(self, x, y):
        n = len(self.path_xy)
        window = 40

        best_idx = 0
        best_dist = float('inf')
        hit_edge = False

        if self.path_is_loop:
            # En bucle la ventana circula: el punto más cercano puede estar
            # "antes" del índice 0 (es decir, al final del arreglo).
            for off in range(-window, window + 1):
                i = (self._nearest_idx_hint + off) % n
                px, py = self.path_xy[i]
                d = (px - x) ** 2 + (py - y) ** 2
                if d < best_dist:
                    best_dist = d
                    best_idx = i
                    hit_edge = (off in (-window, window))
        else:
            start = max(0, self._nearest_idx_hint - window)
            end = min(n, self._nearest_idx_hint + window)
            best_idx = start
            for i in range(start, end):
                px, py = self.path_xy[i]
                d = (px - x) ** 2 + (py - y) ** 2
                if d < best_dist:
                    best_dist = d
                    best_idx = i
            hit_edge = best_idx in (start, end - 1)

        if hit_edge:
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
        cuya distancia en línea recta a `idx` sea >= lookahead_dist.
        En modo bucle la búsqueda da la vuelta por el índice 0, así que
        el controlador nunca se queda sin puntos hacia adelante."""
        n = len(self.path_xy)
        x0, y0 = self.path_xy[idx]

        if self.path_is_loop:
            for step in range(1, n):
                j = (idx + step) % n
                xj, yj = self.path_xy[j]
                if math.hypot(xj - x0, yj - y0) >= lookahead_dist:
                    return j
            return (idx + 1) % n

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
        if self.path_is_loop:
            j = (idx + self.lookahead_curvature_pts) % n
        else:
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
        traveled = 0.0
        steps = 0
        j = idx
        while traveled < lookahead_m and steps < n - 1:
            kappa = self._estimate_curvature(j)
            if kappa > max_kappa:
                max_kappa = kappa
            nxt = (j + 1) % n if self.path_is_loop else j + 1
            if not self.path_is_loop and nxt > n - 1:
                break
            px, py = self.path_xy[j]
            qx, qy = self.path_xy[nxt]
            traveled += math.hypot(qx - px, qy - py)
            j = nxt
            steps += 1
        return max_kappa

    def _publish_zero_commands(self):
        self.steering_pub.publish(Float32(data=0.0))
        self.throttle_pub.publish(Float32(data=0.0))
        self._prev_steering_norm = 0.0
        self._prev_throttle = 0.0

    def _speed_from_history(self, now_sec):
        """Velocidad = desplazamiento entre la pose más antigua y la más
        reciente de la ventana, dividido por el tiempo que las separa.

        Dos detalles importantes:

        - Calcularla como dist/dt entre DOS poses CONSECUTIVAS la vuelve muy
          ruidosa: con /ips irregular, dos muestras separadas 50 ms dan un
          valor disparatado. Ese ruido pasaba al PID, que encendía y apagaba
          el throttle (el avance a tirones).

        - Sumar la longitud de cada segmento tampoco sirve: como cada
          |segmento| es positivo, el ruido de posición se ACUMULA en vez de
          cancelarse y la velocidad queda sesgada hacia arriba. Usando el
          desplazamiento entre los extremos, el ruido sólo afecta a dos
          puntos y se reparte sobre toda la ventana.

        En curva el desplazamiento en línea recta subestima ligeramente el
        arco recorrido, pero para ventanas de ~1 s a estas velocidades la
        diferencia es de pocos puntos porcentuales: mucho menor que el ruido
        que elimina.
        """
        while (len(self._pose_history) > 2 and
               (now_sec - self._pose_history[0][0]) > self.speed_window_sec):
            self._pose_history.popleft()

        if len(self._pose_history) < 2:
            return self._speed_estimate

        t0, x0, y0 = self._pose_history[0]
        t1, x1, y1 = self._pose_history[-1]
        span = t1 - t0
        if span < 1e-3:
            return self._speed_estimate
        return math.hypot(x1 - x0, y1 - y0) / span

    def _update_speed_estimate(self, x, y, now_sec):
        """Estima la velocidad del vehículo a partir de poses sucesivas.

        Es el punto más delicado del nodo: /ips publica a ~1.4 Hz mientras
        el TF se republica a 20 Hz, así que la mayoría de ciclos ven la
        MISMA posición. Distinguir "no llegó muestra nueva" de "el auto
        está detenido" es lo que hace que el lazo de velocidad funcione.
        """
        min_move_eps = 0.01  # metros
        STALE_TIMEOUT = 2.0  # segundos

        self._pose_is_fresh = False

        if self._last_pos is None or self._last_time is None:
            self._last_pos = (x, y)
            self._last_time = now_sec
            self._last_real_update_time = now_sec
            self._pose_history.clear()
            self._pose_history.append((now_sec, x, y))
            return

        dist = math.hypot(x - self._last_pos[0], y - self._last_pos[1])

        if dist > min_move_eps:
            dt_speed = now_sec - self._last_time

            # El umbral de salto escala con el tiempo transcurrido: con /ips
            # a ~1.4Hz un salto legítimo puede ser de varios metros. Sólo es
            # teletransporte si la velocidad implícita es imposible.
            plausible_jump = self.max_plausible_speed * max(dt_speed, 0.05)
            is_teleport = dist > max(2.0, plausible_jump * 1.5)

            if is_teleport:
                self._pose_history.clear()
                self._pose_history.append((now_sec, x, y))
                self._speed_estimate = 0.0
                self.steer_pid.reset()
                self.speed_pid.reset()
                self._last_pid_correction = 0.0
                self._nearest_idx_hint = 0
            else:
                self._pose_history.append((now_sec, x, y))
                self._speed_estimate = self._speed_from_history(now_sec)
                self._pose_is_fresh = True

            self._last_pos = (x, y)
            self._last_time = now_sec
            self._last_real_update_time = now_sec

        elif (self._last_real_update_time is not None and
                (now_sec - self._last_real_update_time) > STALE_TIMEOUT):
            # Sin movimiento durante más tiempo del que tarda /ips: el auto
            # está realmente detenido, no es que falte la muestra.
            self._pose_history.append((now_sec, x, y))
            self._speed_estimate = self._speed_from_history(now_sec)
            self._pose_is_fresh = True

    def _control_loop(self):
        if not self.path_ready or len(self.path_xy) < 2:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.vehicle_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            # Sin esta advertencia el nodo salía de cada ciclo en silencio y
            # parecía "colgado": cargaba la ruta y no volvía a imprimir nada.
            # La causa habitual es que sim_tf_broadcaster_node no está
            # corriendo, o que el bridge todavía no publica /ips e /imu.
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if (now_sec - self._last_tf_error_log) >= 2.0:
                self._last_tf_error_log = now_sec
                self.get_logger().warn(
                    f'Sin transformada {self.map_frame} -> {self.vehicle_frame}: {exc}. '
                    f'¿Está corriendo sim_tf_broadcaster_node y publicando el bridge?')
            self._publish_zero_commands()
            return

        tf_age = self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)
        if tf_age.nanoseconds * 1e-9 > self.max_tf_age_sec:
            # La transformada map->vehicle no se ha actualizado en un
            # buen rato (p. ej. el bridge/sim se desconectó o el TF
            # broadcaster se quedó republicando datos viejos). Seguir
            # publicando comandos sobre una pose obsoleta sería conducir
            # a ciegas, así que se corta throttle/steering y se espera.
            self._publish_zero_commands()
            self._recovering_from_stale = True
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

        if self._recovering_from_stale:
            # Volvió la pose después de un corte. Durante el corte el auto
            # siguió moviéndose "a ciegas", así que su heading real puede
            # haber divergido mucho respecto de la ruta y el índice guardado
            # ya no sirve. Se descarta el estado acumulado para evitar que
            # el primer ciclo tras la recuperación produzca un latigazo de
            # dirección a máxima autoridad.
            self._recovering_from_stale = False
            self._nearest_idx_hint = 0
            self.steer_pid.reset()
            self.speed_pid.reset()
            self._last_pid_correction = 0.0
            self._pose_history.clear()
            self._pose_history.append((now_sec, x, y))
            self._speed_estimate = 0.0
            self._last_pos = (x, y)
            self._last_time = now_sec
            self._last_real_update_time = now_sec

        self._update_speed_estimate(x, y, now_sec)

        # ---- Compensación de latencia (dead reckoning) ----
        # La pose que acabamos de leer puede tener hasta ~0.7s de antigüedad
        # (/ips publica a ~1.4Hz). Controlar sobre ella significa apuntar a
        # donde el auto ESTUVO, no donde está: a 1.3 m/s son ~0.9 m de
        # desfase, más que el lookahead completo. Aquí proyectamos la pose
        # hacia adelante con el modelo de bicicleta antes de calcular el
        # comando de dirección.
        x_ctrl, y_ctrl, yaw_ctrl = x, y, yaw
        pose_age = 0.0
        if self.latency_compensation and self._last_real_update_time is not None:
            pose_age = min(now_sec - self._last_real_update_time,
                           self.max_prediction_sec)
            if pose_age > 1e-3 and self._speed_estimate > 1e-3:
                # Ángulo real de dirección a partir del último comando
                # normalizado publicado (steering_norm = sign * ang / max).
                steer_angle_prev = (self._prev_steering_norm
                                    * self.max_steering_angle_rad
                                    * self.steering_sign)
                yaw_rate = (self._speed_estimate
                            * math.tan(steer_angle_prev) / self.wheelbase_m)
                yaw_ctrl = normalize_angle(yaw + yaw_rate * pose_age)
                # Se integra con el rumbo medio del intervalo para que la
                # proyección siga el arco y no una recta, que es donde más
                # se notaba el error en curva.
                yaw_mid = normalize_angle(yaw + 0.5 * yaw_rate * pose_age)
                travel = self._speed_estimate * pose_age
                x_ctrl = x + travel * math.cos(yaw_mid)
                y_ctrl = y + travel * math.sin(yaw_mid)

        if self._speed_estimate < self.stall_speed_threshold:
            if self._stall_since is None:
                self._stall_since = now_sec
            elif (now_sec - self._stall_since) > self.stall_timeout:
                self.steer_pid.reset()
                self.speed_pid.reset()
                self._last_pid_correction = 0.0
                self._stall_since = now_sec
        else:
            self._stall_since = None

        idx, _ = self._find_nearest_index(x_ctrl, y_ctrl)
        path_yaw = self.path_yaw[idx]

        cross_track_error = self.cross_track_sign * self._signed_cross_track_error(
            x_ctrl, y_ctrl, idx)
        heading_error = normalize_angle(path_yaw - yaw_ctrl)

        # k_cte efectivo: se refuerza en curvas cerradas (curvatura local alta)
        # para dar más autoridad de corrección ahí, sin desestabilizar los
        # tramos rectos (donde curvature ~ 0 y k_cte se queda en su base).
        curvature = self._estimate_curvature(idx)
        effective_k_cte = self.k_cte + self.k_cte_curve_gain * curvature

        dt = self.control_period

        # ---------------- Ley de dirección ----------------
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
            dx_l = tx - x_ctrl
            dy_l = ty - y_ctrl
            local_x = math.cos(yaw_ctrl) * dx_l + math.sin(yaw_ctrl) * dy_l
            local_y = -math.sin(yaw_ctrl) * dx_l + math.cos(yaw_ctrl) * dy_l
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

        # ---------------- Ley de velocidad ----------------
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

        # El PID de velocidad SÓLO se actualiza cuando llegó una pose nueva.
        # Con el lazo a 10Hz y /ips a ~1.4Hz, actualizarlo en cada ciclo
        # significaba integrar ~7 veces el mismo error viejo: eso es lo que
        # llevaba el throttle a 1.000 con el auto todavía detenido.
        if self._pose_is_fresh:
            self._last_pid_correction = self.speed_pid.update(speed_error, dt)

        # Feedforward: aporta el punto de operación (throttle necesario para
        # sostener target_speed) y el PID sólo corrige la diferencia. Sin
        # esto todo el esfuerzo recaía en el integral, que es el término más
        # lento y el más propenso a windup.
        throttle_ff = self.throttle_ff_per_mps * target_speed
        throttle_raw = throttle_ff + self._last_pid_correction
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

        if self.debug_logging and (now_sec - self._last_debug_log) >= self.debug_period:
            self._last_debug_log = now_sec
            self.get_logger().info(
                f'idx={idx:4d} | e_y={cross_track_error:+.3f} m | e_head={heading_error:+.3f} rad | '
                f'steer_cmd={steering_norm:+.3f} | v_est={self._speed_estimate:.2f} | '
                f'v_target={target_speed:.2f} | curv={braking_curvature:.2f} | '
                f'thr={throttle:.3f} (ff={throttle_ff:.3f} pid={self._last_pid_correction:+.3f}) | '
                f'age={pose_age:.2f}s | lap={self.lap_count}'
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
                    self._lap_times.append(elapsed)

                    es_mejor = (self.best_lap_time is None or
                                elapsed < self.best_lap_time)
                    if es_mejor:
                        self.best_lap_time = elapsed

                    # La marca solo se muestra cuando se bate un tiempo
                    # anterior, no en la primera vuelta (que es "mejor"
                    # por no tener con qué compararse).
                    marca = '  ⭐ mejor tiempo' if (
                        es_mejor and self.lap_count > 1) else ''

                    self.get_logger().info(
                        f'🏁 Vuelta {self.lap_count} completada | '
                        f'tiempo: {elapsed:.3f} s{marca}')

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