import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class Perception(Node):
    """Processes LaserScan and publishes closest obstacle distance."""

    def __init__(self) -> None:
        super().__init__("perception")
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.publisher_ = self.create_publisher(String, "perception_output", 10)
        self.get_logger().info("Perception node started")

    def scan_callback(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        closest = min(msg.ranges)
        out = String()
        out.data = f"{closest:.3f}"
        self.publisher_.publish(out)
        self.get_logger().debug(f"Perception: closest obstacle {closest:.3f} m")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
