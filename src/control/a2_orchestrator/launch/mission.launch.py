"""
Autonomous mission launch: DLIO (optional), mission orchestrator, detection (optional).

Prerequisites (must already be running):
  Sim:  a2 sim
  Real: ros2 launch a2_ros nuc.launch.py + pc2_bridge.sh

Usage:
  a2 mission save_dir:=/tmp/run1
  ros2 launch a2_orchestrator mission.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    a2_orchestrator_dir = get_package_share_directory('a2_orchestrator')
    a2_ros_dir = get_package_share_directory('a2_ros')
    a2_ros_launch_dir = os.path.join(a2_ros_dir, 'launch')
    mission_defaults = os.path.join(
        a2_orchestrator_dir, 'config', 'mission_defaults.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    include_dlio = LaunchConfiguration('include_dlio')
    save_dir = LaunchConfiguration('save_dir')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (/clock)',
        ),
        DeclareLaunchArgument(
            'include_dlio',
            default_value='true',
            description='Start DLIO in this launch (set false if already running)',
        ),
        DeclareLaunchArgument(
            'sim_detection',
            default_value='false',
            description='Use sim object_detection launch (uncompressed camera)',
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='/tmp/a2_mission',
            description='Directory for mission outputs (map, origin, detections)',
        ),
        DeclareLaunchArgument(
            'skip_home',
            default_value='false',
            description='Skip return navigation after map save',
        ),
        DeclareLaunchArgument(
            'exploration_timeout_sec',
            default_value='600.0',
            description='Max exploration duration in seconds',
        ),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='/camera/image/compressed',
            description='Camera topic for orchestrator prereq check (sim: /camera/image_raw)',
        ),
    ]

    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a2_ros_launch_dir, 'dlio.launch.py'),
        ),
        condition=IfCondition(include_dlio),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_crop_enabled': 'false',
            'rviz': 'false',
        }.items(),
    )

    # Object detection disabled until ONNX model path is fixed in object_detection package.
    # detect_real_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('object_detection'),
    #             'launch',
    #             'object_detection_real.launch.py',
    #         ),
    #     ),
    #     condition=UnlessCondition(LaunchConfiguration('sim_detection')),
    #     launch_arguments={
    #         'object_detection_classes': '[39]',
    #         'debayer_image': 'false',
    #     }.items(),
    # )
    #
    # detect_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('object_detection'),
    #             'launch',
    #             'object_detection.launch.py',
    #         ),
    #     ),
    #     condition=IfCondition(LaunchConfiguration('sim_detection')),
    #     launch_arguments={
    #         'object_detection_classes': '[39]',
    #     }.items(),
    # )
    #
    # detection_logger = Node(
    #     package='a2_orchestrator',
    #     executable='detection_logger',
    #     name='detection_logger',
    #     output='screen',
    #     parameters=[{
    #         'output_csv': PathJoinSubstitution([save_dir, 'detections.csv']),
    #         'map_frame': 'map',
    #         'detection_info_topic': '/detection_info',
    #     }],
    # )

    sim_time_param = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    mission_orchestrator = Node(
        package='a2_orchestrator',
        executable='mission_orchestrator',
        name='mission_orchestrator',
        output='screen',
        parameters=[
            mission_defaults,
            sim_time_param,
            {
                'save_dir': save_dir,
                'camera_image_topic': LaunchConfiguration('camera_image_topic'),
                'skip_home': ParameterValue(
                    LaunchConfiguration('skip_home'), value_type=bool
                ),
                'exploration_timeout_sec': ParameterValue(
                    LaunchConfiguration('exploration_timeout_sec'), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            dlio_launch,
            # detect_real_launch,
            # detect_sim_launch,
            # detection_logger,
            mission_orchestrator,
        ]
    )
