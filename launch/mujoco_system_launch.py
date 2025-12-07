from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=["/path/to/config/mujoco_hardware.yaml"]
        )
    ])
