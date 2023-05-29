from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rocket_visual',
            executable='static_broad',
            name='fixed_frame'
        ),
        Node(
            package='rocket_visual',
            executable='broad',
            name='rocket_broadcaster',
            parameters=[
                {'rocket_frame': 'rocket1'}
            ]
        ),
    ])