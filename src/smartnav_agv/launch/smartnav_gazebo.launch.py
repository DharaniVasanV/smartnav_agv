from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch TurtleBot3 Gazebo world (acts as our AGV platform)
    tb3_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_world.launch.py",
    )

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_launch)
    )

    # SmartNav AGV nodes
    fleet_bridge = Node(
        package="smartnav_agv",
        executable="fleet_bridge",
        output="screen",
    )

    task_manager = Node(
        package="smartnav_agv",
        executable="task_manager",
        output="screen",
    )

    perception = Node(
        package="smartnav_agv",
        executable="perception",
        output="screen",
    )

    global_planner = Node(
        package="smartnav_agv",
        executable="global_planner",
        output="screen",
    )

    local_planner = Node(
        package="smartnav_agv",
        executable="local_planner",
        output="screen",
    )

    controller = Node(
        package="smartnav_agv",
        executable="controller",
        output="screen",
    )

    motor_driver = Node(
        package="smartnav_agv",
        executable="motor_driver",
        output="screen",
    )

    return LaunchDescription(
        [
            turtlebot3_gazebo,
            fleet_bridge,
            task_manager,
            perception,
            global_planner,
            local_planner,
            controller,
            motor_driver,
        ]
    )
