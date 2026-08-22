import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class SimTfBroadcasterNode(Node):
    """
    Publica la transformada map -> f1tenth_1 combinando:
      - /autodrive/f1tenth_1/ips  (geometry_msgs/Point): posición ground-truth
      - /autodrive/f1tenth_1/imu  (sensor_msgs/Imu): orientación (quaternion)

    Esto permite ver el vehículo del simulador correctamente ubicado
    dentro del mapa/`Path` en RViz, ya que el bridge de AutoDRIVE no
    publica esta transformada por sí mismo.
    """

    def __init__(self):
        super().__init__('sim_tf_broadcaster_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'f1tenth_1')
        self.declare_parameter('broadcast_rate_hz', 20.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('broadcast_rate_hz').value

        self.last_position = None   # geometry_msgs/Point
        self.last_orientation = None  # geometry_msgs/Quaternion

        self.ips_sub = self.create_subscription(
            Point, '/autodrive/f1tenth_1/ips', self.ips_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/autodrive/f1tenth_1/imu', self.imu_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.broadcast_tf)

        self.get_logger().info(
            f'SimTfBroadcasterNode listo. Publicando {self.map_frame} -> '
            f'{self.base_frame} a {rate} Hz (cuando haya datos).')

    def ips_callback(self, msg: Point):
        self.last_position = msg

    def imu_callback(self, msg: Imu):
        self.last_orientation = msg.orientation

    def broadcast_tf(self):
        if self.last_position is None or self.last_orientation is None:
            return  # Todavía no llega suficiente info del simulador

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.last_position.x
        t.transform.translation.y = self.last_position.y
        t.transform.translation.z = self.last_position.z

        t.transform.rotation = self.last_orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimTfBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()