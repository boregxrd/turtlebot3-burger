import rclpy
# importamos las librerias ROS2 de python
from rclpy.node import Node
# importamos los mensajes tipo Twist
from geometry_msgs.msg import Twist
import math

# creamos una clase pasándole como parámetro el Nodo
class SimplePublisher(Node):

    def __init__(self):
        # Constructor de la clase
        # ejecutamos super() para inicializar el Nodo
        # introducimos le nombre del nodo como parámetro
        super().__init__('simple_publisher')
        # creamos el objeto publisher
        # que publicara en el topic /cmd_vel
        # la cola del topic es de 10 mensajes
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # definimos un periodo para publicar periodicamente
        timer_period = 0.5
        # creamos un timer con dos parametros:
        # - el periodo (0.5 seconds)
        # - la funcion a realizar  (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.radius = 2.0 # circunferencia 2 metros de radio
        self.linear_velocity = 0.5
        self.angular_velocity = self.linear_velocity/self.radius # ω = v / r

        self.total_rotation = 0.0
        self.target_rotation = 4*math.pi # una vuelta 2pi, 2 vueltas 4pi

    def timer_callback(self):
        if self.total_rotation < self.target_rotation:
            # creamos el mensaje tipo Twist
            msg = Twist()
            # define la velocidad lineal en el eje x
            msg.linear.x = self.linear_velocity
            # define la velocidad angular en el eje z
            msg.angular.z = self.angular_velocity
            # Publicamos el mensaje en el topic
            self.publisher_.publish(msg)
            # Mostramos el mensaje por el terminal
            self.get_logger().info('Publishing: "%s"' % msg)

            self.total_rotation += self.angular_velocity*self.timer_period
        else:
            self.stop_robot()

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Robot stopped.')
        self.timer.cancel() #detener el temporizador para que no siga publicando

def main(args=None):
    # inicializa la comunicación
    rclpy.init(args=args)
    # declara el constructor del nodo
    simple_publisher = SimplePublisher()
    # dejamos vivo el nodo
    # para parar el programa habrá que matar el node (ctrl+c)
    rclpy.spin(simple_publisher)
    # destruye en nodo
    simple_publisher.destroy_node()
    # se cierra la comunicacion ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()