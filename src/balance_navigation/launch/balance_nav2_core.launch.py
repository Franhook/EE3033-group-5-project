import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_pkg_dir = get_package_share_directory('balance_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Nav2 Nav2 navigation-framework core
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(nav_pkg_dir, 'config', 'nav2_core_params.yaml'),
            'use_velocity_smoother': 'true' 
        }.items()
    )

    return LaunchDescription([
        # Force control of velocity commands
        SetRemap(src='cmd_vel_nav', dst='cmd_vel'),
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel'), 
        nav2_launch
    ])
