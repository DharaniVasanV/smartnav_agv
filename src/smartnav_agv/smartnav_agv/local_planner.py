import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LocalPlanner(Node):
    """Combines global route and perception into motion commands."""

    def __init__(self) -> None:
        super().__init__("local_planner")
        self.route_sub = self.create_subscription(
            String, "global_plan_output", self.route_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, "perception_output", self.perception_callback, 10
        )
        self.publisher_ = self.create_publisher(String, "motion_commands", 10)

        self.current_route = None
        self.closest_obstacle = None
        self.obstacle_threshold = 0.5  # meters

        self.get_logger().info("LocalPlanner node started")

    def route_callback(self, msg: String) -> None:
        self.current_route = msg.data
        self.get_logger().info(f"LocalPlanner: updated route: {self.current_route}")

    def perception_callback(self, msg: String) -> None:
        try:
            self.closest_obstacle = float(msg.data)
        except Exception:
            self.get_logger().warn("LocalPlanner: cannot parse perception message")
            return

        command = String()
        if self.closest_obstacle < self.obstacle_threshold:
            command.data = "STOP"
        else:
            command.data = "MOVE_FORWARD"

        self.publisher_.publish(command)
        self.get_logger().info(
            f"LocalPlanner: closest={self.closest_obstacle:.2f} -> {command.data}"
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
