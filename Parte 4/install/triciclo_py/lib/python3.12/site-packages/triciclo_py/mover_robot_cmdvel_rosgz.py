# archivo: mover_robot_cmdvel_rosgz.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TricicloMoverRosGz(Node):
    def __init__(self):
        super().__init__('triciclo_mover_rosgz')

        # Publicadores a cada joint del robot en Gazebo Harmonic (ros_gz)
        self.pub_izq = self.create_publisher(Float64,
            '/world/default/model/triciclo/joint/propulsion_izq/cmd_vel', 10)
        self.pub_der = self.create_publisher(Float64,
            '/world/default/model/triciclo/joint/propulsion_der/cmd_vel', 10)
        self.pub_dir = self.create_publisher(Float64,
            '/world/default/model/triciclo/joint/direccion/cmd_vel', 10)

        # Suscribirse a /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Nodo de movimiento listo y esperando comandos en /cmd_vel.')

    def cmd_vel_callback(self, msg: Twist):
        # Extraer velocidades lineales y angulares
        vel_lineal = msg.linear.x
        vel_giro = msg.angular.z

        # Publicar velocidades a las ruedas y la dirección
        self.pub_izq.publish(Float64(data=vel_lineal))
        self.pub_der.publish(Float64(data=vel_lineal))
        self.pub_dir.publish(Float64(data=vel_giro))

def main(args=None):
    rclpy.init(args=args)
    nodo = TricicloMoverRosGz()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

