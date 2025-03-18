# simple_node.py
import time
import rclpy
# importamos el modulo Node de la librería ROS2 de python
from rclpy.node import Node

#definimos una funcion basica

def main(args=None):
    #inicializamos la comunicacion ROS
    rclpy.init(args=args)
    #creamos un nodo
    my_node = Node('my_node')
    try:
        #imprimimos un mensaje por el terminal
        my_node.get_logger().info('Los miembros de mi equipo son:')
        # Aqui tienes que poner tu propio codigo
        # crea un diccionario con los nombres de los integrantes
        manu = {'nombre':'Manuel','apellido':'Borregales'}
        juan = {'nombre':'Juan','apellido':'Diaz'}
        alex = {'nombre':'Alex','apellido':'Rosado'}
        marito = {'nombre':'Mario','apellido':'Luis'}
        integrantes = {
            'Manuel':manu,
            'Juan':juan,
            'Alex':alex,
            'Mario':marito
        }
        while True:
            for integrante in integrantes:
                my_node.get_logger().info(f'{integrante} {integrantes[integrante]["nombre"]}')
                my_node.get_logger().info(f'{integrante} {integrantes[integrante]["apellido"]}')
                my_node.get_logger().info('----------waiting-----------')  
                time.sleep(2)
                # Dejamos abierto el nodo hasta pulsar ctrl+c
                rclpy.spin_once(my_node, timeout_sec=2)

    except KeyboardInterrupt:
        my_node.get_logger().info('Cerrando el nodo')
    finally:
        #destruimos el nodo
        my_node.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()

#definimos el ejecutable
if __name__=='__main__':
  main() # llamada a la funcion