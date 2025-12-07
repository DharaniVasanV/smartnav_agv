import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskManager(Node):
    """Receives commands from fleet bridge and issues navigation goals."""

    def __init__(self) -> None:
        super().__init__("task_manager")
        self.subscription = self.create_subscription(
            String, "fleet_commands", self.receive_command, 10
        )
        self.goal_publisher = self.create_publisher(String, "navigation_goal", 10)
        self.get_logger().info("TaskManager node started")

    def receive_command(self, msg: String) -> None:
        self.get_logger().info(f"TaskManager: received command: {msg.data}")
        goal = String()
        goal.data = "GO_TO:station_A"
        self.goal_publisher.publish(goal)
        self.get_logger().info("TaskManager: published navigation goal")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
