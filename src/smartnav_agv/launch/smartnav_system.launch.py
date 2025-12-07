from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="smartnav_agv", executable="fleet_bridge", output="screen"),
            Node(package="smartnav_agv", executable="task_manager", output="screen"),
            Node(package="smartnav_agv", executable="perception", output="screen"),
            Node(package="smartnav_agv", executable="global_planner", output="screen"),
            Node(package="smartnav_agv", executable="local_planner", output="screen"),
            Node(package="smartnav_agv", executable="controller", output="screen"),
            Node(package="smartnav_agv", executable="motor_driver", output="screen"),
        ]
    )
