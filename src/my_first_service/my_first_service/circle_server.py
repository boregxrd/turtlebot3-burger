"""
ROS 2 Package: my_first_service
Author: Manuel Borregales
Date: 20/03

This package provides a ROS 2 service and client for controlling circular movement
of a robot. The service sets velocity commands, while the client monitors odometry
to complete a full circular trajectory before stopping the robot.

This is the server node that:

Sets the velocity for a circular trajectory (or stops). 
It does not itself monitor how long or how far the robot should keep moving. 
The client node will handle that by subscribing to /odom and deciding when to stop.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interface.srv import CircleMovement

class CircleService(Node):
    """
    ROS 2 Service Node for controlling circular movement.
    This service receives movement parameters (speed, radius, direction) and
    publishes velocity commands accordingly.
    """
    def __init__(self):
        super().__init__('circle_service')

        self.srv = self.create_service(
            CircleMovement,
            'circle_movement',
            self.circle_callback
        )

        # Publisher to cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel_msg_ = Twist()

        self.get_logger().info('Servidor circle_service inicializado')

    def circle_callback(self, request, response):
        """
        Callback for handling incoming service requests.
        Adjusts the robot's velocity based on the requested parameters.
        """
        if request.speed == 0.0 or request.radius == 0.0:
            self.vel_msg_.linear.x = 0.0
            self.vel_msg_.angular.z = 0.0
            self.get_logger().info('Deteniendo el robot')
        else:
            vx = float(request.speed)
            wz = float(request.speed / request.radius)
            if request.direction == "left":
                self.vel_msg_.linear.x = vx
                self.vel_msg_.angular.z = +wz
                self.get_logger().info(f'Moviendo en círculo a la IZQUIERDA (R={request.radius:.2f} m)')
            elif request.direction == "right":
                self.vel_msg_.linear.x = vx
                self.vel_msg_.angular.z = -wz
                self.get_logger().info(f'Moviendo en círculo a la DERECHA (R={request.radius:.2f} m)')
            else:
                self.vel_msg_.linear.x = 0.0
                self.vel_msg_.angular.z = 0.0
                self.get_logger().warn('Dirección no válida, robot parado')
        # Publish the command
        self.publisher_.publish(self.vel_msg_)
        response.success = True
        return response


def main(args=None):
    """
    Main entry point for the circle_service node.
    Initializes the ROS 2 node and starts spinning.
    """
    rclpy.init(args=args)
    node = CircleService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando circle_service...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
