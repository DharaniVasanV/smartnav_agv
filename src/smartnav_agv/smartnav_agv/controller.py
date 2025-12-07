import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Controller(Node):
    """Converts motion commands into /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("controller")
        self.subscription = self.create_subscription(
            String, "motion_commands", self.command_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Controller node started")

    def command_callback(self, msg: String) -> None:
        twist = Twist()
        if msg.data == "MOVE_FORWARD":
            twist.linear.x = 0.8
        else:
            twist.linear.x = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(
            f"Controller: command={msg.data} -> linear.x={twist.linear.x:.2f}"
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
