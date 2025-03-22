from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pinocchio_robot',
            executable='pinocchio_node',
            name='pinocchio_test_node',
            output='screen'
        )
    ])
