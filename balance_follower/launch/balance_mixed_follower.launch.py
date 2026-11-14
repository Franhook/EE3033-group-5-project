import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'balance_follower'
    pkg_share = get_package_share_directory(pkg_name)
    
    config_path = os.path.join(pkg_share, 'config', 'params_mixed.yaml')

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='follower_mixed_node',
            name='follower_mixed_node',
            output='screen',
            parameters=[config_path],
            # remappings=[
            #     ('/scan', '/lslidar_scan')
            # ]
        )
    ])
