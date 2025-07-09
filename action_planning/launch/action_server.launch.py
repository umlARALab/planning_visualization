from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    share_dir = get_package_share_directory('action_planning')
    moveit_demo_launch_file = PathJoinSubstitution([get_package_share_directory('moveit2_tutorials'), 'launch', 'demo.launch.py'])


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_demo_launch_file)
        ),
        Node(
            package='action_planning',
            executable='action_server',
            name="action_server"
        ),
        # Node(
        #     package='action_planning',
        #     executable='point_pub',
        #     name="point_pub"
        # )
    ])