#!/usr/bin/env python3
"""STEP 1 - build the map on site (autonomous exploration or by hand).

    ros2 launch balance_bringup balance_slam_nav_explore_bringup.launch.py

Arguments
    explore       true  -> explore_lite drives the car (default)
                  false -> no explorer; drive with teleop (allowed in Step 1)
    map_save_path where the finished map is written (no extension)
                  default ~/maps/sandbox_map  ->  sandbox_map.yaml / .pgm

Start sequence (each stage waits for the real thing, not for a timer):
    base -> /odom -> LiDAR -> /scan -> resampler -> /scan_fixed
         -> SLAM -> /map -> Nav2 ACTIVE + TF(map->base) -> explore_lite
         -> mapping supervisor (saves the map when exploring finishes)

HOLD THE CAR when the base starts: the STM32 resets on node start-up.
Put the car at the assigned START position first.  SLAM makes that spot the
map origin (0, 0), which Step 2 relies on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def waiter(label, shell_condition, timeout_s=120):
    """Run until ``shell_condition`` succeeds (or the timeout expires)."""
    script = f"""
    echo '[bringup] Waiting for {label}...'
    end=$((SECONDS + {timeout_s}))
    until {shell_condition}
    do
        if [ $SECONDS -ge $end ]; then
            echo '[bringup] WARNING: timed out waiting for {label}; continuing'
            exit 0
        fi
        sleep 1
    done
    echo '[bringup] {label} OK'
    """
    return ExecuteProcess(cmd=['bash', '-c', script], output='screen')


def topic_ready(topic, reliability=None, durability=None):
    cmd = f'timeout 5 ros2 topic echo {topic} --once'
    if reliability:
        cmd += f' --qos-reliability {reliability}'
    if durability:
        cmd += f' --qos-durability {durability}'
    return waiter(f'topic {topic}', f'{cmd} >/dev/null 2>&1')


def nav2_ready():
    # bt_navigator can fail to load its behaviour tree if the planner's global
    # costmap services are not up yet; the lifecycle manager then aborts and
    # leaves nodes unconfigured/inactive.  Wait, and if Nav2 is still not
    # ACTIVE after a grace period, bring the nodes up by hand in order.
    script = """
    echo '[bringup] Waiting for Nav2 lifecycle nodes ACTIVE...'
    state() { ros2 lifecycle get "$1" 2>/dev/null | awk 'NR==1{print $1}'; }
    NODES="/planner_server /controller_server /smoother_server /behavior_server /bt_navigator"
    start=$SECONDS
    while true; do
        all=1
        for n in /planner_server /controller_server /bt_navigator; do
            [ "$(state $n)" = "active" ] || all=0
        done
        if [ $all -eq 1 ]; then echo '[bringup] Nav2 lifecycle nodes ACTIVE OK'; exit 0; fi
        if [ $((SECONDS - start)) -ge 180 ]; then
            echo '[bringup] WARNING: Nav2 not active after 180 s; continuing'; exit 0
        fi
        if [ $((SECONDS - start)) -ge 20 ]; then
            for n in $NODES; do
                st=$(state $n)
                if [ "$st" = "unconfigured" ]; then
                    echo "[bringup] recovering $n (configure)"
                    ros2 lifecycle set $n configure >/dev/null 2>&1
                    st=$(state $n)
                fi
                if [ "$st" = "inactive" ]; then
                    echo "[bringup] recovering $n (activate)"
                    ros2 lifecycle set $n activate >/dev/null 2>&1
                fi
            done
        fi
        sleep 3
    done
    """
    return ExecuteProcess(cmd=['bash', '-c', script], output='screen')


def include(package, *path, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), *path)),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    declare = [
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument(
            'map_save_path',
            default_value=os.path.expanduser('~/maps/sandbox_map')),
    ]

    robot_base = include(
        'balance_robot_base', 'launch', 'balance_robot_base.launch.py')
    lidar = include('lslidar_driver_n10p', 'launch', 'lslidar_launch.py')
    slam = include(
        'balance_mapping', 'launch',
        'balance_mapping_slamtoolbox_async.launch.py')
    nav2 = include('balance_navigation', 'launch', 'balance_nav2_core.launch.py')

    # Installed script, started as a normal ROS node (no path guessing).
    resampler = Node(
        package='balance_mapping',
        executable='scan_resampler.py',
        name='scan_resampler',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_fixed',
            'number_of_bins': 450,
        }],
    )

    supervisor = Node(
        package='balance_mission',
        executable='mapping_supervisor_node',
        name='mapping_supervisor',
        output='screen',
        parameters=[{'map_save_path': LaunchConfiguration('map_save_path')}],
    )

    wait_odom = topic_ready('/odom')
    wait_scan = topic_ready('/scan', reliability='best_effort')
    wait_scan_fixed = topic_ready('/scan_fixed', reliability='best_effort')
    wait_map = topic_ready('/map', durability='transient_local')
    wait_nav2 = nav2_ready()

    def after(waiting, actions):
        return RegisterEventHandler(
            OnProcessExit(target_action=waiting, on_exit=actions))

    return LaunchDescription(declare + [
        LogInfo(msg='[bringup] HOLD THE CAR; start it on the START spot.'),
        robot_base,
        wait_odom,
        after(wait_odom, [lidar, wait_scan]),
        after(wait_scan, [resampler, wait_scan_fixed]),
        after(wait_scan_fixed, [slam, wait_map]),
        after(wait_map, [nav2, wait_nav2]),
        after(wait_nav2, [
            LogInfo(msg='[bringup] Starting map supervisor / explorer.'),
            supervisor,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('balance_mapping'),
                    'launch', 'balance_explorer_lite.launch.py')),
                condition=IfCondition(LaunchConfiguration('explore')),
            ),
        ]),
    ])
