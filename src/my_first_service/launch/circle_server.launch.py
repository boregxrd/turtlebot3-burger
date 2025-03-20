from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_service',
            executable='circle_server',
            output='screen'
        ),
        Node(
            package='my_first_service',
            executable='circle_movement',
            output='screen',
            parameters=[
                {'radius': 1.0},
                {'speed': 0.15},
                {'direction': 'left'}
            ]
        )
    ])