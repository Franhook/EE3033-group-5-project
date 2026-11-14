from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('strategy', default_value='random', description='Patrol strategy: random or spin'),
        Node(
            package='balance_navigation',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[{'strategy': LaunchConfiguration('strategy')}]
        )
    ])
