import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('balance_usb_cam')
    
    # Define the input argument mode; use default by default default
    mode_arg = DeclareLaunchArgument('mode', default_value='default')
    
    # Construct the configuration filename：camera_params_ + mode + .yaml
    config_file = [
        os.path.join(pkg_share, 'config', 'camera_params_'),
        LaunchConfiguration('mode'),
        '.yaml'
    ]
    
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[config_file],
        remappings=[('/image_raw', '/image_raw')]
    )
    
    return LaunchDescription([mode_arg, usb_cam_node])
