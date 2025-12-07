import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FleetBridge(Node):
    """Simulated bridge to a higher-level fleet or cloud system."""

    def __init__(self) -> None:
        super().__init__("fleet_bridge")
        self.publisher_ = self.create_publisher(String, "fleet_commands", 10)
        self.timer = self.create_timer(2.0, self.send_heartbeat)
        self.get_logger().info("FleetBridge node started")

    def send_heartbeat(self) -> None:
        msg = String()
        msg.data = "heartbeat"
        self.publisher_.publish(msg)
        self.get_logger().debug("FleetBridge: heartbeat sent")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
