from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('visualization')
    # tcp_connector_launch_file = PathJoinSubstitution([get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.launch.py'])

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(tcp_connector_launch_file)
        # ),
        Node(
            package='visualization',
            executable='stretch_ar_transforms',
            name='stretch_ar_transforms',
            output='screen'
        ),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='default_server_endpoint',
            parameters=[{'ROS_TCP_PORT':10000}, {'ROS_IP':'192.168.10.3'}],
            output='log'
        )
    ])

# ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.10.3 -p ROS_TCP_PORT:=10000
#
