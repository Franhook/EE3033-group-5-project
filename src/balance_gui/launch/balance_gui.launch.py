import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Parameters passed from the terminal
    use_ssh_arg = DeclareLaunchArgument('use_ssh', default_value='False', description='Enable SSH remote control')
    remote_ip_arg = DeclareLaunchArgument('remote_ip', default_value='balance.local', description='Target Raspberry Pi IP')
    remote_user_arg = DeclareLaunchArgument('remote_user', default_value='balance', description='SSH Username')
    remote_pwd_arg = DeclareLaunchArgument('remote_pwd', default_value='PWD_030609', description='SSH Password')
    use_docker_arg = DeclareLaunchArgument('use_docker', default_value='False', description='Is ROS 2 inside Docker?')
    local_pwd_arg = DeclareLaunchArgument('local_pwd', default_value='dongguan', description='Local working directory (for NFS)')

    # gui node
    gui_node = Node(
        package='balance_gui',
        executable='balance_gui',
        name='balance_gui_node',
        output='screen',
        parameters=[{
            'use_ssh': LaunchConfiguration('use_ssh'),
            'remote_ip': LaunchConfiguration('remote_ip'),
            'remote_user': LaunchConfiguration('remote_user'),
            'remote_pwd': LaunchConfiguration('remote_pwd'),
            'use_docker': LaunchConfiguration('use_docker'),
            'local_pwd': LaunchConfiguration('local_pwd'),
            'docker_name': 'ros2_humble'
        }]
    )

    return LaunchDescription([
        use_ssh_arg,
        remote_ip_arg,
        remote_user_arg,
        remote_pwd_arg,
        use_docker_arg,
        local_pwd_arg,
        gui_node
    ])

# ros2 run balance_gui balance_gui --ros-args -p remote_ip:="192.168.0.101" -p use_ssh:=true
