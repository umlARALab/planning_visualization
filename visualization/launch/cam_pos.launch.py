from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    stretch_driver_launch_file = PathJoinSubstitution([get_package_share_directory('stretch_core'), 'launch', 'stretch_driver.launch.py'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stretch_driver_launch_file)
        ),
        Node(
            package='visualization',
            executable='aruco_detection',
            name='aruco_detection',
            output='log'
        ),
        Node(
            package='web_video_server',
            executable='aruco_node',
            name='aruco_node',
            output='log'
        )
    ])
