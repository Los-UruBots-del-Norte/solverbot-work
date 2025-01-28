import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from motor_board_md49.motor_board import MotorBoardMD49


class MotorBoardNode(Node):
    def __init__(self):
        super().__init__('motor_board_node')

        # Configuración del puerto serie
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)

        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value

        # Inicializar la clase MotorBoardMD49
        self.motor_board = MotorBoardMD49(port, baudrate)

        # Suscribirse al tópico /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publicar la velocidad de los motores
        self.speed1_publisher = self.create_publisher(Int32, '/motor_speed_1', 10)
        self.speed2_publisher = self.create_publisher(Int32, '/motor_speed_2', 10)

        # Timer para leer datos periódicamente
        self.timer = self.create_timer(0.1, self.publish_motor_speeds)

        self.get_logger().info("MotorBoardNode inicializado")

    def cmd_vel_callback(self, msg: Twist):
        linear = int(msg.linear.x * 100)  # Escalar velocidades
        angular = int(msg.angular.z * 100)

        self.motor_board.SetSpeed1(linear)
        self.motor_board.set_speed_2_turn(angular)

    def publish_motor_speeds(self):
        speed1 = self.motor_board.GetSpeed1()
        speed2 = self.motor_board.GetSpeed2()

        if speed1 is not None:
            self.speed1_publisher.publish(Int32(data=speed1))
        if speed2 is not None:
            self.speed2_publisher.publish(Int32(data=speed2))

    def destroy_node(self):
        self.motor_board.Close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MotorBoardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando el nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()