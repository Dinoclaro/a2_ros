"""
Navigation stack launch for A2 simulation.

Starts the CMU Autonomous Exploration stack on top of the running sim:
  - autonomy_base  : terrain analysis + local planner + path follower
  - far_planner    : global visibility-graph planner

Prerequisites (provided by sim.launch.py + a2_bridge):
  /state_estimation  - ground-truth odometry (published by a2_bridge_sim in a2_unitree_bridge)
  /registered_scan   - world-frame lidar cloud (published by a2_bridge_sim in a2_unitree_bridge)
  /clock             - sim time clock (published by a2_bridge_sim in a2_unitree_bridge)

Usage:
  # Terminal 1
  ros2 launch a2_ros sim.launch.py

  # Terminal 2 (after sim is up)
  ros2 launch a2_ros navigation.launch.py use_sim_time:=true

  # Then bring the robot up to locomotion. These go through the /a2/set_mode
  # service (via the `a2` CLI), which reports whether each transition was
  # accepted by the FSM:
  a2 stand    # mode 2: stand up
  a2 unlock   # mode 3: unlock joints (balance stand)
  a2 walk     # mode 4: locomotion

  # Send a navigation goal in RViz using the 'Goalpoint' button,
  # or publish directly:
  ros2 topic pub /way_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'odom'}, point: {x: 5.0, y: 0.0, z: 0.0}}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    a2_ros_dir = get_package_share_directory('a2_ros')
    a2_ros_launch_dir = os.path.join(a2_ros_dir, 'launch')
    rviz_path = os.path.join(a2_ros_dir, 'rviz', 'navigation.rviz')
    far_config = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with navigation config',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (/clock)',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    autonomy_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a2_ros_launch_dir, 'autonomy_base.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autonomy_speed': '1.0',
        }.items(),
    )

    far_planner = Node(
        package='far_planner',
        executable='far_planner',
        name='far_planner',
        output='screen',
        additional_env={'QT_QPA_PLATFORM': 'offscreen'},
        parameters=[
            far_config,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ('/odom_world', '/state_estimation'),
            ('/terrain_cloud', '/terrain_map_ext'),
            ('/scan_cloud', '/registered_scan'),
            ('/terrain_local_cloud', '/terrain_map'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        autonomy_base,
        far_planner,
        rviz,
    ])
