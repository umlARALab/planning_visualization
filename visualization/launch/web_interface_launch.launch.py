from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('visualization')
    stretch_driver_launch_file = PathJoinSubstitution([get_package_share_directory('stretch_core'), 'launch', 'stretch_driver.launch.py'])
    stretch_camera_launch_file = PathJoinSubstitution([get_package_share_directory('stretch_core'), 'launch', 'd435i_high_resolution.launch.py'])
    rosbridge_launch_file = PathJoinSubstitution([get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stretch_driver_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stretch_camera_launch_file)
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_file),
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port':8080}, {'address':'192.168.10.5'}],
            output='log'
        )
    ])