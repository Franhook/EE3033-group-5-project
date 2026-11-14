import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'balance_mapping'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Point to the configuration file for the synchronous version
    slam_config_path = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=slam_config_path,
        description='Path to sync slam_toolbox config file'
    )

    # Define the slam_toolbox node
    slam_toolbox_node = Node(
        parameters=[LaunchConfiguration('slam_config'), {'use_sim_time': False}],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        slam_config_arg,
        slam_toolbox_node
    ])
