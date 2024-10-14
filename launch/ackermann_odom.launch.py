from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_odom',
            executable='ackermann_odom_node',
            name='ackermann_odom',
            output='screen',
            parameters=[],
        ),
    ])