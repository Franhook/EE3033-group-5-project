import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mapping_pkg_dir = get_package_share_directory('balance_mapping')
    
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[os.path.join(mapping_pkg_dir, 'config', 'explore_params.yaml')]
    )

    return LaunchDescription([explore_node])
