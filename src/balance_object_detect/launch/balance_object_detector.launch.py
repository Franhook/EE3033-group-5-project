import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'balance_object_detect'
    pkg_share = get_package_share_directory(pkg_name)

    # --- 1. Declare parameters ---
    
    # Camera-configuration path
    config_path = os.path.join(pkg_share, 'config', 'params_detector.yaml')
    arg_config = DeclareLaunchArgument(
        'config', default_value=config_path,
        description='Path to params_detector.yaml file'
    )
    
    # YOLO YOLO model
    arg_yolo = DeclareLaunchArgument(
        'yolo_model', default_value='yolo26n.pt',
        description='YOLO model name (e.g., yolo11n.pt, yolo11s.pt)'
    )
    
    # ReID ReID model (for people))
    arg_reid = DeclareLaunchArgument(
        'reid_model', default_value='osnet_x0_25',
        description='Person ReID model name (e.g., osnet_x0_25)'
    )
    
    # Generic Generic model (for general objects))
    arg_generic = DeclareLaunchArgument(
        'generic_model', default_value='mobilenet_v3_small',
        description='Generic feature extractor (mobilenet_v3_small or mobilenet_v3_large)'
    )

    # Debugging interface
    arg_view = DeclareLaunchArgument(
        'rqt_view', default_value='False',
        description='Whether to launch rqt_image_view'
    )

    # --- 2. Define nodes ---
    # Core target-detection node
    node_detector = Node(
        package=pkg_name,
        executable='object_detector',
        name='object_detector_node',
        output='screen',
        parameters=[{
            'yolo_model': LaunchConfiguration('yolo_model'),
            'reid_model': LaunchConfiguration('reid_model'),
            'generic_model': LaunchConfiguration('generic_model'),
        }, LaunchConfiguration('config')] # Also load parameters from the config file
    )

    # RQT RQT visualization node (optional))
    node_rqt = Node(
        condition=IfCondition(LaunchConfiguration('rqt_view')),
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/detector/result_image']
    )

    return LaunchDescription([
        arg_config,
        arg_yolo,
        arg_reid,
        arg_generic,
        arg_view,
        node_detector,
        node_rqt
    ])