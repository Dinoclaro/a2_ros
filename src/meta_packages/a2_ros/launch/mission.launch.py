"""
Autonomous mission launch: explore until target detected, return home, save map.

Prerequisites (must already be running):
  Sim:
    a2 sim                    # mission launch includes DLIO — do NOT also run a2 dlio
    If you use a2 sim --dlio, pass include_dlio:=false to this launch.
  Real robot:
    ros2 launch a2_pc2 pc2.launch.py
    ros2 launch a2_ros nuc.launch.py

This launch starts:
  - DLIO odometry + mapping (unless include_dlio:=false)
  - mission_orchestrator + waypoint mux immediately
  - Object detection, autonomy, TARE, far_planner after a short delay so stand-up
    is not starved by heavy node startup

Usage (sim):
  # Terminal 1
  a2 sim

  # Terminal 2
  a2 mission target_class:=bottle save_dir:=/tmp/mission_sim

Usage (real robot):
  ros2 launch a2_ros mission.launch.py target_class:=bottle save_dir:=/tmp/mission_001
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    a2_ros_dir = get_package_share_directory('a2_ros')
    a2_ros_launch_dir = os.path.join(a2_ros_dir, 'launch')
    mission_defaults = os.path.join(a2_ros_dir, 'config', 'mission', 'mission_defaults.yaml')
    tare_mission_config = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_mission.yaml')
    far_config = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    include_dlio = LaunchConfiguration('include_dlio')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (/clock)',
        ),
        DeclareLaunchArgument(
            'include_dlio',
            default_value='true',
            description='Start DLIO in this launch (set false if already running, e.g. a2 sim --dlio + a2 dlio)',
        ),
        DeclareLaunchArgument(
            'sim_detection',
            default_value='false',
            description='Use sim object_detection launch (uncompressed camera)',
        ),
        DeclareLaunchArgument(
            'target_class',
            default_value='bottle',
            description='YOLO class name to trigger homing (matches detection_info.class_id)',
        ),
        DeclareLaunchArgument(
            'object_detection_classes',
            default_value='[39]',
            description='COCO class IDs passed to object detection',
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='/tmp/a2_mission',
            description='Directory for DLIO map + detection exports',
        ),
        DeclareLaunchArgument(
            'map_crop_enabled',
            default_value='false',
            description='DLIO map cropping (disable for full mission maps)',
        ),
        DeclareLaunchArgument(
            'debayer_image',
            default_value='false',
            description='Debayer camera images before detection (real robot)',
        ),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='/camera/image/compressed',
            description='Camera topic for orchestrator prereq check (sim: /camera/image_raw)',
        ),
        DeclareLaunchArgument(
            'planning_delay_sec',
            default_value='8.0',
            description='Seconds to wait before starting detection/planning nodes (lets robot stand)',
        ),
    ]

    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a2_ros_launch_dir, 'dlio.launch.py'),
        ),
        condition=IfCondition(include_dlio),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_crop_enabled': LaunchConfiguration('map_crop_enabled'),
            'rviz': 'false',
        }.items(),
    )

    detect_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'object_detection_real.launch.py',
            ),
        ),
        condition=UnlessCondition(LaunchConfiguration('sim_detection')),
        launch_arguments={
            'object_detection_classes': LaunchConfiguration('object_detection_classes'),
            'debayer_image': LaunchConfiguration('debayer_image'),
        }.items(),
    )

    detect_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'object_detection.launch.py',
            ),
        ),
        condition=IfCondition(LaunchConfiguration('sim_detection')),
        launch_arguments={
            'object_detection_classes': LaunchConfiguration('object_detection_classes'),
        }.items(),
    )

    autonomy_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a2_ros_launch_dir, 'autonomy_base.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    sim_time_param = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    tare_planner = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        parameters=[tare_mission_config, sim_time_param],
    )

    far_planner = Node(
        package='far_planner',
        executable='far_planner',
        name='far_planner',
        output='screen',
        additional_env={'QT_QPA_PLATFORM': 'offscreen'},
        parameters=[far_config, sim_time_param],
        remappings=[
            ('/odom_world', '/state_estimation'),
            ('/terrain_cloud', '/terrain_map_ext'),
            ('/scan_cloud', '/registered_scan'),
            ('/terrain_local_cloud', '/terrain_map'),
            ('/way_point', '/far/way_point'),
        ],
    )

    waypoint_mux = Node(
        package='a2_orchestrator',
        executable='waypoint_mux',
        name='waypoint_mux',
        output='screen',
        parameters=[mission_defaults, sim_time_param],
    )

    mission_orchestrator = Node(
        package='a2_orchestrator',
        executable='mission_orchestrator',
        name='mission_orchestrator',
        output='screen',
        parameters=[
            mission_defaults,
            sim_time_param,
            {
                'target_class': LaunchConfiguration('target_class'),
                'save_dir': LaunchConfiguration('save_dir'),
                'camera_image_topic': LaunchConfiguration('camera_image_topic'),
            },
        ],
    )

    # Defer heavy CPU nodes so stand-up is not competing with YOLO/TARE startup.
    delayed_planning = TimerAction(
        period=LaunchConfiguration('planning_delay_sec'),
        actions=[
            detect_real_launch,
            detect_sim_launch,
            autonomy_base,
            tare_planner,
            far_planner,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            dlio_launch,
            waypoint_mux,
            mission_orchestrator,
            delayed_planning,
        ]
    )
