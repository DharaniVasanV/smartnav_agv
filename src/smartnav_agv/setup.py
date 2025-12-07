from setuptools import setup

package_name = 'smartnav_agv'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/smartnav_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Motion Detectors',
    maintainer_email='dharanivasan.v2024csbs@sece.ac.in',
    description='SmartNav AGV - ROS2 navigation architecture for hackathon',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_bridge = smartnav_agv.fleet_bridge:main',
            'task_manager = smartnav_agv.task_manager:main',
            'perception = smartnav_agv.perception:main',
            'global_planner = smartnav_agv.global_planner:main',
            'local_planner = smartnav_agv.local_planner:main',
            'controller = smartnav_agv.controller:main',
            'motor_driver = smartnav_agv.motor_driver_node:main',
        ],
    },
)

