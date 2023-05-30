import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory("rocket_visual")

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'rocket_model.urdf.xml'
    urdf = os.path.join(
        pkg_path,
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_file_name = 'rviz_basic.rviz'
    rviz_config = os.path.join(
        pkg_path,
        rviz_config_file_name)

    return LaunchDescription([
        
    #    Node(
    #        package='rocket_visual',
    #        executable='broad_serial',
    #        name='rocket_broadcaster',
    #        parameters=[
    #            {'rocket_frame': 'rocket1'}
    #        ]
    #    ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='rocket_visual',
            executable='static_broad',
            name='fixed_frame'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_rocket',
            arguments = ['-d', rviz_config],
        )
    ])