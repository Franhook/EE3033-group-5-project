import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('balance_object_detect'),
        'config', 'params_combined_target.yaml',
    )
    return LaunchDescription([Node(
        package='balance_object_detect', executable='combined_target',
        name='object_detector_node', output='screen',
        parameters=[config],
    )])
