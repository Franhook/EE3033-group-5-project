#!/usr/bin/env python3
"""STEP 2 - autonomous mission on the map saved in Step 1.

    ros2 launch balance_mission balance_mission_bringup.launch.py \
        map:=/home/ros2_ws/maps/sandbox_map.yaml

Then, once the target has been placed and the car is on the start spot:

    ros2 service call /mission/start std_srvs/srv/Trigger

(or add  autostart:=true  to start by itself when everything is ready).

Everything is started with fixed delays; the mission manager itself waits for
Nav2 to be ACTIVE, TF and the map, so starting early is harmless.

Arguments
    map            saved map yaml (default ~/maps/sandbox_map.yaml)
    autostart      start the mission automatically (default false)
    with_camera    start the USB camera node (default true)
    with_detector  start YOLO detector on THIS machine (default true).
                   Set false if the detector runs on the laptop instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, *path):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), *path)
        )
    )


def delayed(seconds, message, actions):
    return TimerAction(
        period=seconds,
        actions=[LogInfo(msg=f'[mission-bringup] {message}')] + actions,
    )


def generate_launch_description():
    mission_share = get_package_share_directory('balance_mission')
    params = os.path.join(mission_share, 'config', 'mission_params.yaml')

    default_map = os.path.expanduser('~/maps/sandbox_map.yaml')

    args = [
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('with_camera', default_value='true'),
        DeclareLaunchArgument('with_detector', default_value='true'),
    ]

    # ---- hardware ---------------------------------------------------
    robot_base = include(
        'balance_robot_base', 'launch', 'balance_robot_base.launch.py')
    lidar = include(
        'lslidar_driver_n10p', 'launch', 'lslidar_launch.py')

    # ---- localization + navigation on the saved map ------------------
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('balance_navigation'),
            'launch', 'balance_amcl_localization.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map')}.items(),
    )
    nav2 = include(
        'balance_navigation', 'launch', 'balance_nav2_core.launch.py')

    # ---- perception ---------------------------------------------------
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('balance_usb_cam'),
            'launch', 'usb_cam.launch.py')),
        launch_arguments={'mode': 'detector'}.items(),
        condition=IfCondition(LaunchConfiguration('with_camera')),
    )

    mission = Node(
        package='balance_mission',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[
            params,
            {'autostart': ParameterValue(
                LaunchConfiguration('autostart'), value_type=bool)},
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg='[mission-bringup] HOLD THE CAR while the base starts '
                    '(the STM32 resets).'),
        robot_base,
        delayed(2.0, 'LiDAR', [lidar]),
        delayed(5.0, 'map server + AMCL', [localization]),
        delayed(9.0, 'Nav2', [nav2]),
        delayed(11.0, 'camera', [camera]),
        delayed(15.0, 'mission manager (waits for Nav2 ACTIVE)', [mission]),
    ])
