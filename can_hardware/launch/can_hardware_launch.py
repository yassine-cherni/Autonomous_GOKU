from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2socketcan',
            executable='can_receiver',
            name='can_receiver',
            parameters=[{'can_iface': 'can0'}]
        ),
        Node(
            package='ros2socketcan',
            executable='can_sender',
            name='can_sender',
            parameters=[{'can_iface': 'can0'}]
        ),
        Node(
            package='my_can_hardware',
            executable='can_hardware_node',
            name='can_hardware_node'
        )
    ])
