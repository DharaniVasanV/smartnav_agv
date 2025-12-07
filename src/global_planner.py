import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GlobalPlanner(Node):

    def __init__(self) -> None:
        super().__init__("global_planner")
        self.subscription = self.create_subscription(
            String, "navigation_goal", self.goal_callback, 10
        )
        self.publisher_ = self.create_publisher(String, "global_plan_output", 10)
        self.get_logger().info("GlobalPlanner node started")

    def goal_callback(self, msg: String) -> None:
        self.get_logger().info(f"GlobalPlanner: goal received: {msg.data}")
        route = String()
        route.data = "GLOBAL_ROUTE: A -> B -> C"
        self.publisher_.publish(route)
        self.get_logger().info("GlobalPlanner: published global route")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
