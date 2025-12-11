# SmartNav AGV

A lightweight ROS2 navigation system for intelligent AGVs, built with real-time perception, planning, and 3D simulation.

## Overview

SmartNav AGV is a modular software system that adds intelligent navigation to existing industrial AGVs.
It reads sensor data, makes movement decisions, avoids obstacles, and moves toward a predefined endpoint.

The system is fully compatible with Gazebo simulation and follows a vendor-agnostic integration model, meaning it can run on any AGV hardware that accepts cmd_vel commands.

## Features

- Real-time obstacle detection using LiDAR (/scan)
- Goal-based movement: robot moves only when a navigation goal is sent
- Endpoint navigation using odometry (/odom)
- Automatic STOP when obstacle is too close or goal is reached
- 3D Simulation using Gazebo + TurtleBot3
- Clear, modular ROS2 nodes for perception, planning, and control

## Architecture (High-Level)

SmartNav AGV consists of the following nodes:
1. **Task Manager** – receives and forwards goals  
2. **Global Planner** – interprets goal commands  
3. **Local Planner** – computes movement based on odometry + perception  
4. **Perception Node** – detects closest obstacle  
5. **Controller** – outputs velocity commands  
6. **Motor Driver** – publishes `/cmd_vel`  
7. **Goal Sender** – triggers robot motion for testing  


## Simulation Setup
1. *Launch Gazebo + SmartNav*
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger  
ros2 launch smartnav_agv smartnav_gazebo.launch.py  

3. *Send a navigation goal*
source /opt/ros/humble/setup.bash
ros2 run smartnav_agv goal_sender  

What happens:
- AGV starts moving forward
- Uses odometry to track progress
- Stops automatically upon reaching the endpoint (~2m)
- Stops immediately if an obstacle is detected

## Build Instructions
cd ~/smartnav_ws  
colcon build --symlink-install  
source install/setup.bash  

## Project Structure
smartnav_agv/  
├── launch/  
├── smartnav_agv/  
│ ├── perception.py  
│ ├── local_planner.py  
│ ├── controller.py  
│ ├── motor_driver_node.py  
│ ├── global_planner.py  
│ ├── task_manager.py  
│ └── goal_sender.py  
├── package.xml  
└── setup.py  

## Team

### Motion Detectors
- Dharani Vasan V
- Maheswara Pandiyan A
- Sudharshan S
- Madan P A

## License
MIT License – free to use, modify, and build upon.
