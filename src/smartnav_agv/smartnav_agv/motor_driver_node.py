import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorDriver(Node):
    """Simulated motor driver (would talk to real hardware in production)."""

    def __init__(self) -> None:
        super().__init__("motor_driver_node")
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.get_logger().info("MotorDriver node started")

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.get_logger().info(
            f"MotorDriver: applying velocity linear.x={msg.linear.x:.2f} m/s"
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
