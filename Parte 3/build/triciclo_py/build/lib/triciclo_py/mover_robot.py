import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TricicloControl(Node):
    def __init__(self):
        super().__init__('triciclo_control')

        # Publicadores para las ruedas y dirección
        self.pub_izq = self.create_publisher(Float64, '/propulsion_izq/command', 10)
        self.pub_der = self.create_publisher(Float64, '/propulsion_der/command', 10)
        self.pub_dir = self.create_publisher(Float64, '/direccion/command', 10)

        # Subscripción a cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)

    def callback_cmd_vel(self, msg):
        vel = msg.linear.x      # avance
        ang = msg.angular.z     # giro

        # Publicar misma velocidad a ambas ruedas traseras
        self.pub_izq.publish(Float64(data=vel))
        self.pub_der.publish(Float64(data=vel))

        # Publicar ángulo de dirección (simple)
        self.pub_dir.publish(Float64(data=ang))

def main(args=None):
    rclpy.init(args=args)
    nodo = TricicloControl()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

