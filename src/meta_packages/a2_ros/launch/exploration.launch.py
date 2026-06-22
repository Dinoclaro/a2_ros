"""
Autonomous exploration launch for A2 simulation using TARE planner.

Starts the full exploration stack on top of the running sim:
  - autonomy_base       : terrain analysis + local planner + path follower
  - tare_planner        : autonomous coverage exploration

Prerequisites (provided by sim.launch.py + a2_bridge):
  /state_estimation  - ground-truth odometry (published by a2_bridge in a2_sim_utils)
  /registered_scan   - world-frame lidar cloud (published by a2_bridge in a2_sim_utils)
  /clock             - sim time clock (published by sim_clock in a2_sim_utils)

Usage:
  # Terminal 1
  ros2 launch a2_ros sim.launch.py scene:=scene_obstacles.xml

  # Terminal 2
  cd src/control/a2_locomotion_controller/scripts
  ./control_mode.sh --stand
  ./control_mode.sh --walk

  # Terminal 3
  ros2 launch a2_ros exploration.launch.py rviz:=true use_sim_time:=true
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
    rviz_path = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    tare_config = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_a2.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2',
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
        }.items(),
    )

    tare_planner = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        parameters=[
            tare_config,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
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
        tare_planner,
        rviz,
    ])
