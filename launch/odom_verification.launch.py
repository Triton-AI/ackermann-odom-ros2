from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    
    bridge_node = Node(
        package='autodrive_f1tenth',
        executable='autodrive_bridge',
        name='autodrive_bridge',
        emulate_tty=True,
        output='screen',
    )

    left_encoder_node = Node(
        package='ackermann_odom',
        executable='encoder_node',
        name='left_encoder',
        output='screen',
        parameters=[
            {'topic_name': 'autodrive/f1tenth_1/left_encoder'},
            {'joint_name': 'left_wheel_joint'}
        ],
    )
    
    right_encoder_node = Node(
        package='ackermann_odom',
        executable='encoder_node',
        name='right_encoder',
        output='screen',
        parameters=[
            {'topic_name': 'autodrive/f1tenth_1/right_encoder'},
            {'joint_name': 'right_wheel_joint'}
        ],
    )

    dummy_steering_node = Node(
        package='ackermann_odom',
        executable='dummy_steering_publisher',
        name='dummy_steering',
        output='screen',
    )
    
    ackermann_odom_node = Node(
        package='ackermann_odom',
        executable='ackermann_odom_node',
        name='ackermann_odom',
        output='screen',
        parameters=[],
    )
    
    ld.add_action(left_encoder_node)
    ld.add_action(right_encoder_node)
    ld.add_action(dummy_steering_node)
    ld.add_action(ackermann_odom_node)
    
    return ld