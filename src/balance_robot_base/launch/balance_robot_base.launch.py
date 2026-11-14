import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get the current config-parameter path
    config_path = LaunchConfiguration('config').perform(context)
    
    # Parse the YAML file
    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)
        
    # --- 1. Dynamically extract all Frame IDs Frame ID ---
    ros_params = config_yaml.get('balance_robot_node', {}).get('ros__parameters', {})
    robot_frame = ros_params.get('robot_frame_id', 'base_footprint')
    laser_frame = ros_params.get('laser_frame_id', 'laser_link')
    camera_frame = ros_params.get('camera_frame_id', 'camera_link')
    gyro_frame = ros_params.get('gyro_frame_id', 'gyro_link')

    # --- 2. Dynamically extract physical TF offsets ---
    #tf_params = config_yaml.get('tf_parameters', {}) #org
    tf_params = ros_params.get('tf_parameters', {})
    laser_pos = tf_params.get('laser', {})
    camera_pos = tf_params.get('camera', {})
    gyro_pos = tf_params.get('gyro', {})

    # Define the main chassis node
    balance_node = Node(
        package='balance_robot_base',
        executable='balance_robot_node',
        name='balance_robot_node',
        output='screen',
        parameters=[config_path],
    )

    # Static TF: dynamic robot_frame -> dynamic laser_frame laser_frame
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=[
            '--x', str(laser_pos.get('x', 0.002)),
            '--y', str(laser_pos.get('y', 0.0)),
            '--z', str(laser_pos.get('z', 0.1875)),
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', robot_frame,
            '--child-frame-id', laser_frame
        ]
    )

    # Static TF: dynamic robot_frame -> dynamic camera_frame camera_frame
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', str(camera_pos.get('x', -0.035)),
            '--y', str(camera_pos.get('y', 0.0)),
            '--z', str(camera_pos.get('z', 0.2655)),
            '--yaw', '0.0',
            '--pitch', str(camera_pos.get('pitch', 0.0)),
            '--roll', '0.0',
            '--frame-id', robot_frame,
            '--child-frame-id', camera_frame
        ]
    )

    # Static TF: dynamic robot_frame -> dynamic gyro_frame gyro_frame
    gyro_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gyro_static_tf',
        arguments=[
            '--x', str(gyro_pos.get('x', -0.02)),
            '--y', str(gyro_pos.get('y', -0.04)),
            '--z', str(gyro_pos.get('z', 0.155)),
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', robot_frame,
            '--child-frame-id', gyro_frame
        ]
    )

    return [balance_node, laser_tf_node, camera_tf_node, gyro_tf_node]


def generate_launch_description():
    pkg_name = 'balance_robot_base'
    pkg_share = get_package_share_directory(pkg_name)

    # Configuration-file path
    config_path = os.path.join(pkg_share, 'config', 'base_params.yaml')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config', 
        default_value=config_path,
        description='Path to parameter config file'
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=launch_setup)
    ])