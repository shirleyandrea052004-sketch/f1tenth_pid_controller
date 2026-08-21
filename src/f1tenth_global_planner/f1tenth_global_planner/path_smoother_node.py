import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.interpolate import CubicSpline


class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother_node')

        self.declare_parameter('points_per_meter', 10.0)
        self.declare_parameter('min_waypoints_for_spline', 4)

        self.points_per_meter = self.get_parameter('points_per_meter').value
        self.min_waypoints = self.get_parameter('min_waypoints_for_spline').value

        # El publisher de /raw_path es latched (transient_local), así que
        # nos suscribimos igual para recibir la última ruta publicada.
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
            f'PathSmootherNode listo. points_per_meter={self.points_per_meter}')

    def raw_path_callback(self, msg: Path):
        n = len(msg.poses)
        if n < self.min_waypoints:
            self.get_logger().warn(
                f'Ruta cruda tiene solo {n} puntos, se necesitan al menos '
                f'{self.min_waypoints} para suavizar. Se omite.')
            return

        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])

        # --- Quitar puntos duplicados consecutivos (rompen el spline) ---
        keep = np.ones(n, dtype=bool)
        for i in range(1, n):
            if abs(xs[i] - xs[i - 1]) < 1e-6 and abs(ys[i] - ys[i - 1]) < 1e-6:
                keep[i] = False
        xs, ys = xs[keep], ys[keep]
        n = len(xs)
        if n < self.min_waypoints:
            self.get_logger().warn('Muy pocos puntos únicos tras filtrar duplicados.')
            return

        # --- Parametrizar por longitud de arco acumulada ---
        deltas = np.hypot(np.diff(xs), np.diff(ys))
        arc_length = np.concatenate(([0.0], np.cumsum(deltas)))
        total_length = arc_length[-1]

        if total_length < 1e-3:
            self.get_logger().warn('Longitud total de la ruta ~0, se omite suavizado.')
            return

        # --- Cubic Spline paramétrico: x(s), y(s) ---
        cs_x = CubicSpline(arc_length, xs)
        cs_y = CubicSpline(arc_length, ys)

        num_samples = max(int(total_length * self.points_per_meter), n * 2)
        s_dense = np.linspace(0.0, total_length, num_samples)
        xs_smooth = cs_x(s_dense)
        ys_smooth = cs_y(s_dense)

        self.publish_smoothed(msg.header, xs_smooth, ys_smooth)

    def publish_smoothed(self, header, xs, ys):
        path_msg = Path()
        path_msg.header.frame_id = header.frame_id or 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(xs)):
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(xs[i])
            pose.pose.position.y = float(ys[i])

            # Orientación tangente a la curva (útil para seguimiento futuro)
            if i < len(xs) - 1:
                dx = xs[i + 1] - xs[i]
                dy = ys[i + 1] - ys[i]
            else:
                dx = xs[i] - xs[i - 1]
                dy = ys[i] - ys[i - 1]
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