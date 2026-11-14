import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'balance_robot_base'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Configuration-file path
    config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # 2. Chassis driver node (STM32 communication))
    driver_node = Node(
        package=pkg_name,
        executable='balance_robot_node',
        name='balance_robot_node',
        output='log', 
        parameters=[config_path],
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    # 3. Keyboard-control node
    # prefix='xterm -e' prefix='xterm -e' is an advanced technique that opens a new terminal window for keyboard control
    # If xterm is not installed on the Raspberry Pi, remove the prefix line and operate directly in the current terminal
    teleop_node = Node(
        package=pkg_name,
        executable='balance_keyboard_control',
        name='balance_keyboard_control',
        output='screen'
        # prefix='xterm -e' # Optional: if xterm is installed, uncomment to run in a pop-up terminal
    )

    return LaunchDescription([
        driver_node,
        teleop_node
    ])
