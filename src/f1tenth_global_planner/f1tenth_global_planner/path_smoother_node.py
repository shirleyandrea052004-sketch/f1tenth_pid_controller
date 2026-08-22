import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.interpolate import PchipInterpolator
from scipy.ndimage import distance_transform_edt


def douglas_peucker_indices(points: np.ndarray, epsilon: float):
    """Devuelve los ÍNDICES (dentro de `points`) que Douglas-Peucker
    conserva, en vez de las coordenadas. Esto nos permite después
    referenciar los puntos crudos originales para hacer refinamiento
    local (agregar puntos intermedios exactamente donde se necesiten)."""
    n = len(points)
    if n < 3:
        return list(range(n))

    def perpendicular_distance(pt, a, b):
        if np.allclose(a, b):
            return np.linalg.norm(pt - a)
        line_vec = b - a
        proj = np.dot(pt - a, line_vec) / np.dot(line_vec, line_vec)
        proj = np.clip(proj, 0.0, 1.0)
        closest = a + proj * line_vec
        return np.linalg.norm(pt - closest)

    keep = {0, n - 1}

    def rdp(start_i, end_i):
        if end_i <= start_i + 1:
            return
        a, b = points[start_i], points[end_i]
        dists = [perpendicular_distance(points[i], a, b)
                 for i in range(start_i + 1, end_i)]
        max_d = max(dists)
        idx = start_i + 1 + dists.index(max_d)
        if max_d > epsilon:
            keep.add(idx)
            rdp(start_i, idx)
            rdp(idx, end_i)

    rdp(0, n - 1)
    return sorted(keep)


class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother_node')

        self.declare_parameter('points_per_meter', 10.0)
        self.declare_parameter('min_waypoints_for_spline', 4)
        self.declare_parameter('simplify_epsilon', 0.15)
        self.declare_parameter('safety_radius', 0.20)
        self.declare_parameter('max_local_refinements', 25)

        self.points_per_meter = self.get_parameter('points_per_meter').value
        self.min_waypoints = self.get_parameter('min_waypoints_for_spline').value
        self.simplify_epsilon = self.get_parameter('simplify_epsilon').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.max_local_refinements = self.get_parameter('max_local_refinements').value

        self.map_info = None
        self.inflated_grid = None
        self.last_raw_msg = None

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        raw_path_qos = QoSProfile(depth=1)
        raw_path_qos.reliability = QoSReliabilityPolicy.RELIABLE
        raw_path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.raw_path_sub = self.create_subscription(
            Path, '/raw_path', self.raw_path_callback, raw_path_qos)

        smoothed_qos = QoSProfile(depth=1)
        smoothed_qos.reliability = QoSReliabilityPolicy.RELIABLE
        smoothed_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.smoothed_pub = self.create_publisher(
            Path, '/smoothed_path', smoothed_qos)

        self.get_logger().info(
            f'PathSmootherNode listo. points_per_meter={self.points_per_meter}, '
            f'safety_radius={self.safety_radius}')

    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        occupied = (grid == 100) | (grid == -1)
        dist = distance_transform_edt(~occupied)
        radius_cells = self.safety_radius / msg.info.resolution
        inflated = grid.copy()
        inflated[dist <= radius_cells] = 100
        self.inflated_grid = inflated

        if self.last_raw_msg is not None:
            self.process_raw_path(self.last_raw_msg)

    def world_to_grid(self, x, y):
        info = self.map_info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return gx, gy

    def invalid_sample_indices(self, xs, ys):
        """Devuelve los índices (dentro de xs/ys) que caen en zona
        ocupada/inflada del grid real."""
        if self.inflated_grid is None:
            return []
        height, width = self.inflated_grid.shape
        bad = []
        for i, (x, y) in enumerate(zip(xs, ys)):
            gx, gy = self.world_to_grid(x, y)
            if not (0 <= gx < width and 0 <= gy < height):
                bad.append(i)
            elif self.inflated_grid[gy, gx] != 0:
                bad.append(i)
        return bad

    # ------------------------------------------------------------------
    def raw_path_callback(self, msg: Path):
        self.last_raw_msg = msg
        self.process_raw_path(msg)

    def process_raw_path(self, msg: Path):
        n = len(msg.poses)
        if n < self.min_waypoints:
            self.get_logger().warn(
                f'Ruta cruda tiene solo {n} puntos, se necesitan al menos '
                f'{self.min_waypoints}. Se omite.')
            return

        xs_raw = np.array([p.pose.position.x for p in msg.poses])
        ys_raw = np.array([p.pose.position.y for p in msg.poses])

        keep = np.ones(n, dtype=bool)
        for i in range(1, n):
            if abs(xs_raw[i] - xs_raw[i - 1]) < 1e-6 and abs(ys_raw[i] - ys_raw[i - 1]) < 1e-6:
                keep[i] = False
        xs_raw, ys_raw = xs_raw[keep], ys_raw[keep]
        if len(xs_raw) < self.min_waypoints:
            self.get_logger().warn('Muy pocos puntos únicos tras filtrar duplicados.')
            return

        raw_points = np.column_stack([xs_raw, ys_raw])

        # --- Simplificación inicial (amplia, para maximizar el suavizado) ---
        control_idx = set(douglas_peucker_indices(raw_points, self.simplify_epsilon))

        xs_smooth, ys_smooth = None, None
        refinements_done = 0
        for attempt in range(self.max_local_refinements + 1):
            ordered_idx = sorted(control_idx)
            simplified = raw_points[ordered_idx]
            xs_smooth, ys_smooth = self.fit_spline(simplified[:, 0], simplified[:, 1])

            bad = self.invalid_sample_indices(xs_smooth, ys_smooth)
            if not bad:
                self.get_logger().info(
                    f'Suavizado válido: {len(ordered_idx)} puntos de control '
                    f'(de {len(raw_points)} crudos) tras {refinements_done} '
                    f'refinamientos locales.')
                break

            # --- Refinamiento LOCAL: solo agregar puntos cerca de los
            # tramos que realmente invaden una pared, sin afectar el
            # resto de la ruta (que puede seguir bien suavizada). ---
            for bi in bad:
                px, py = xs_smooth[bi], ys_smooth[bi]
                nearest_raw_idx = int(np.argmin(
                    (raw_points[:, 0] - px) ** 2 + (raw_points[:, 1] - py) ** 2))
                control_idx.add(nearest_raw_idx)
            refinements_done += 1
        else:
            self.get_logger().warn(
                'No se logró una curva 100% válida con refinamiento local; '
                'se publica la ruta cruda sin suavizar como respaldo.')
            xs_smooth, ys_smooth = xs_raw, ys_raw

        self.publish_smoothed(msg.header, xs_smooth, ys_smooth)

    def fit_spline(self, xs, ys):
        deltas = np.hypot(np.diff(xs), np.diff(ys))
        arc_length = np.concatenate(([0.0], np.cumsum(deltas)))
        total_length = arc_length[-1]

        if total_length < 1e-3:
            return xs, ys

        # PCHIP: shape-preserving, no genera overshoot en giros cerrados
        # (a diferencia de un Cubic Spline "natural").
        cs_x = PchipInterpolator(arc_length, xs)
        cs_y = PchipInterpolator(arc_length, ys)

        num_samples = max(int(total_length * self.points_per_meter), len(xs) * 2)
        s_dense = np.linspace(0.0, total_length, num_samples)
        return cs_x(s_dense), cs_y(s_dense)

    # ------------------------------------------------------------------
    def publish_smoothed(self, header, xs, ys):
        path_msg = Path()
        path_msg.header.frame_id = header.frame_id or 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(xs)):
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(xs[i])
            pose.pose.position.y = float(ys[i])

            if i < len(xs) - 1:
                dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
            else:
                dx, dy = xs[i] - xs[i - 1], ys[i] - ys[i - 1]
            yaw = np.arctan2(dy, dx)
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)

            path_msg.poses.append(pose)

        self.smoothed_pub.publish(path_msg)
        self.get_logger().info(
            f'Ruta suavizada publicada en /smoothed_path con '
            f'{len(path_msg.poses)} puntos.')


def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()