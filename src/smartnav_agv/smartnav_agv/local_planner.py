import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class LocalPlanner(Node):
    """
    Local planner for SmartNav AGV.

    Behaviors:
    - Waits until a navigation goal is received (AGV stays still).
    - Uses odometry (/odom) to measure distance from start pose.
    - Moves forward until a fixed target distance (endpoint) is reached.
    - Stops early if an obstacle is too close (perception input).
    """

    def __init__(self) -> None:
        super().__init__("local_planner")

        self.route_sub = self.create_subscription(
            String, "global_plan_output", self.route_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, "perception_output", self.perception_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.publisher_ = self.create_publisher(String, "motion_commands", 10)

        self.current_route = None        
        self.closest_obstacle = None     
        self.start_x = None              
        self.start_y = None                
        self.current_distance = None       

        self.target_distance = 4.0         

        
        self.obstacle_threshold = 0.5      

        self.get_logger().info(
            f"[LocalPlanner] started with target_distance={self.target_distance} m"
        )

    def route_callback(self, msg: String) -> None:
        """
        Called when a global planner publishes a new route / goal.
        """
        self.current_route = msg.data
        
        self.start_x = None
        self.start_y = None
        self.current_distance = None

        self.get_logger().info(
            f"[LocalPlanner] New route received: {self.current_route}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """
        Track distance traveled from the starting pose using /odom.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        
        if self.start_x is None or self.start_y is None:
            self.start_x = x
            self.start_y = y
            self.current_distance = 0.0
            self.get_logger().info(
                f"[LocalPlanner] Start pose set: x={self.start_x:.2f}, y={self.start_y:.2f}"
            )
            return

        dx = x - self.start_x
        dy = y - self.start_y
        self.current_distance = math.sqrt(dx * dx + dy * dy)

    def perception_callback(self, msg: String) -> None:
        
        if self.current_route is None:
            return

        
        try:
            self.closest_obstacle = float(msg.data)
        except Exception:
            self.get_logger().warn("[LocalPlanner] Cannot parse perception message")
            return

        command = String()

        
        if self.current_distance is None:
            command.data = "MOVE_FORWARD"
            self.publisher_.publish(command)
            self.get_logger().info(
                f"[LocalPlanner] No odom yet -> {command.data}"
            )
            return

        
        if self.current_distance >= self.target_distance:
            command.data = "STOP"
            self.publisher_.publish(command)
            self.get_logger().info(
                f"[LocalPlanner] Goal reached at {self.current_distance:.2f} m -> STOP"
            )
            return

        
        if self.closest_obstacle < self.obstacle_threshold:
            command.data = "STOP"
            self.publisher_.publish(command)
            self.get_logger().info(
                f"[LocalPlanner] Obstacle at {self.closest_obstacle:.2f} m -> STOP"
            )
            return

        
        command.data = "MOVE_FORWARD"
        self.publisher_.publish(command)
        self.get_logger().info(
            f"[LocalPlanner] distance={self.current_distance:.2f} m, "
            f"closest_obstacle={self.closest_obstacle:.2f} m -> {command.data}"
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

