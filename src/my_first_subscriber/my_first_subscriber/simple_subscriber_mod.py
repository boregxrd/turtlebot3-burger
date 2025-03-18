import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point  # Importar el tipo de mensaje Point
from rclpy.qos import ReliabilityPolicy, QoSProfile

class SimpleSubscriberMod(Node):

    def __init__(self):
        super().__init__('simple_subscriber_mod')
        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber  # Evitar warning por variable no usada

    def listener_callback(self, msg):
        # Extraer la posición (x, y) del mensaje Odometry
        position: Point = msg.pose.pose.position
        x, y = position.x, position.y
        self.get_logger().info(f'Posición del robot: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber_mod = SimpleSubscriberMod()
    rclpy.spin(simple_subscriber_mod)
    simple_subscriber_mod.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
