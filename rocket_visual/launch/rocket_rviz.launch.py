from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rocket_visual',
            executable='broad_serial',
            name='rocket_broadcaster',
            parameters=[
                {'rocket_frame': 'rocket1'}
            ]
        ),
        Node(
            package='rocket_visual',
            executable='static_broad',
            name='fixed_frame'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_rocket'
        )
    ])