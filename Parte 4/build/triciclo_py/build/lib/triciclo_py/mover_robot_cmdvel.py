import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import ApplyJointEffort
from builtin_interfaces.msg import Duration

class TricicloMover(Node):
    def __init__(self):
        super().__init__('triciclo_mover')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.cli_izq = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')
        self.cli_der = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')
        self.cli_dir = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')

        while not self.cli_izq.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio Gazebo...')
        self.get_logger().info('Servicio conectado.')

    def cmd_vel_callback(self, msg):
        esfuerzo_lineal = msg.linear.x * 1.0
        esfuerzo_giro = msg.angular.z * 0.5

        # Aplicar fuerza en las ruedas traseras
        self.enviar_esfuerzo('propulsion_izq', esfuerzo_lineal)
        self.enviar_esfuerzo('propulsion_der', esfuerzo_lineal)

        # Aplicar esfuerzo de giro en dirección (horquilla)
        self.enviar_esfuerzo('direccion', esfuerzo_giro)

    def enviar_esfuerzo(self, joint_name, esfuerzo):
        req = ApplyJointEffort.Request()
        req.joint_name = joint_name
        req.effort = esfuerzo
        req.start_time = Duration(sec=0)
        req.duration = Duration(sec=0.1)

        future = self.cli_izq.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    nodo = TricicloMover()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

