import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GoalSender(Node):
    """
    Simple goal sender for SmartNav AGV.
    - After a short delay, publishes a single navigation_goal.
    - This triggers the LocalPlanner to start moving toward the endpoint.
    """

    def __init__(self) -> None:
        super().__init__("goal_sender")
        self.publisher_ = self.create_publisher(String, "navigation_goal", 10)
        self.timer = self.create_timer(3.0, self.send_goal_once)
        self.sent = False
        self.get_logger().info("[GoalSender] Node started, will send goal in 3 seconds")

    def send_goal_once(self) -> None:
        if self.sent:
            return
        msg = String()
        msg.data = "GO_TO:default_endpoint"
        self.publisher_.publish(msg)
        self.sent = True
        self.get_logger().info(
            f"[GoalSender] Published navigation goal: {msg.data}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
