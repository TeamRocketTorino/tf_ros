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

    rviz_config_file_name_1 = 'altitude.rviz'
    rviz_config_1 = os.path.join(
        pkg_path,
        rviz_config_file_name_1)
    
    rviz_config_file_name_2 = 'attitude.rviz'
    rviz_config_2 = os.path.join(
        pkg_path,
        rviz_config_file_name_2)

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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='rocket_visual',
            executable='static_broad',
            name='static_broadcaster'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '-1.57', '--frame-id', 'fixed_frame', '--child-frame-id', 'vertical_frame']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_altitude',
            arguments = ['-d', rviz_config_1],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_attitude',
            arguments = ['-d', rviz_config_2],
        )
    ])