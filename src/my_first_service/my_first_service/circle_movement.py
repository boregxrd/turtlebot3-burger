# my_first_service/circle_movement.py 
# could also be called circle_client.py

# this is the client node that:

# Subscribes to /odom to detect position in each callback.

# Calls the new "circle_movement" service once to start moving in a circle.
# 
# Uses the odometry to detect when the robot has traveled one full lap 
# around the circle. We demonstrate two typical ways to do this:
# 
#   Accumulating distance traveled until it reaches 2 * π * R.
#   Or checking orientation changes (accumulating yaw) until 2π.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from custom_interface.srv import CircleMovement

import math

class CircleMovementClient(Node):
    def __init__(self):
        super().__init__('circle_movement_client')

        # --- SUBSCRIBER: /odom ---
        self.odom_sub_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # --- SERVICE CLIENT: circle_movement ---
        self.client_ = self.create_client(CircleMovement, 'circle_movement')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio circle_movement...')

        # Request para el servicio
        self.req_ = CircleMovement.Request()

        # Variables para controlar el recorrido
        self.start_x_ = None
        self.start_y_ = None
        self.dist_traveled_ = 0.0
        self.last_x_ = None
        self.last_y_ = None
        self.circle_done_ = False

        # Podemos declarar parámetros si queremos:
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('direction', 'left')

        # Leerlos:
        self.radius_ = self.get_parameter('radius').get_parameter_value().double_value
        self.speed_ = self.get_parameter('speed').get_parameter_value().double_value
        self.direction_ = self.get_parameter('direction').get_parameter_value().string_value

        # Circunferencia total a recorrer
        self.full_circle_distance_ = 2.0 * math.pi * self.radius_

        # Llamar al servicio para iniciar el movimiento
        self.send_request(self.radius_, self.speed_, self.direction_)

        self.get_logger().info(
            f'Iniciando trayectoria circular: R={self.radius_:.2f} m, '
            f'vel={self.speed_:.2f} m/s, dir={self.direction_}'
        )

    def send_request(self, radius, speed, direction):
        """
        Envía la petición al servicio circle_movement para
        que el robot se ponga a girar con los parámetros deseados.
        """
        self.req_.radius = float(radius)
        self.req_.speed = float(speed)
        self.req_.direction = direction

        self.future_ = self.client_.call_async(self.req_)

    def odom_callback(self, msg):
        """
        Recibe la odometría, calcula la distancia recorrida
        y, cuando completamos una vuelta, paramos el robot.
        """
        # Extraer la posición (x, y)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Inicializar la posición de partida si es la primera vez
        if self.start_x_ is None:
            self.start_x_ = x
            self.start_y_ = y
            self.last_x_ = x
            self.last_y_ = y
            return

        # Calcular distancia incremental desde la última posición
        dx = x - self.last_x_
        dy = y - self.last_y_
        inc_dist = math.sqrt(dx * dx + dy * dy)
        self.dist_traveled_ += inc_dist

        # Actualizar la última posición
        self.last_x_ = x
        self.last_y_ = y

        # Imprimir posición actual
        self.get_logger().info(
            f'Pos actual: x={x:.2f}, y={y:.2f}, Dist. recorrida={self.dist_traveled_:.2f} m'
        )

        # ¿Hemos completado el círculo?
        if (not self.circle_done_) and (self.dist_traveled_ >= self.full_circle_distance_):
            self.get_logger().info('¡¡Trayectoria circular completada!!')
            self.circle_done_ = True
            self.stop_robot()

    def stop_robot(self):
        """
        Llama al servicio con velocidad = 0 para detener el robot
        y cancela la suscripción al topic /odom.
        """
        stop_req = CircleMovement.Request()
        stop_req.radius = 0.0     # no importa
        stop_req.speed = 0.0
        stop_req.direction = "left"  # tampoco importa, con speed=0 paramos
        self.client_.call_async(stop_req)
        self.get_logger().info('Robot detenido')

        # Unsubscribe from /odom so that no further callbacks occur
        self.destroy_subscription(self.odom_sub_)


def main(args=None):
    rclpy.init(args=args)
    node = CircleMovementClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando circle_movement_client...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()