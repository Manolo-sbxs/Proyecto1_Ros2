import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DemoMovimiento(Node):
    def __init__(self):
        super().__init__('demo_movimiento')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Iniciando demo de movimiento...')
        self.timer = self.create_timer(1.0, self.ejecutar_demo)
        self.paso = 0

    def ejecutar_demo(self):
        msg = Twist()

        if self.paso == 0:
            self.get_logger().info('Avanzando...')
            msg.linear.x = 1.0
        elif self.paso == 1:
            self.get_logger().info('Girando...')
            msg.angular.z = 0.8
        elif self.paso == 2:
            self.get_logger().info('Detenido.')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            self.destroy_timer(self.timer)
            return

        self.publisher.publish(msg)
        self.paso += 1

def main(args=None):
    rclpy.init(args=args)
    nodo = DemoMovimiento()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

