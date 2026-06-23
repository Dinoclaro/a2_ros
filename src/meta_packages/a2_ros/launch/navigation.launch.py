"""
Navigation stack launch for A2 simulation.

Starts the CMU Autonomous Exploration stack on top of the running sim:
  - terrain_analysis     : builds /terrain_map from /registered_scan + /state_estimation
  - terrain_analysis_ext : builds /terrain_map_ext (global terrain for far_planner)
  - local_planner        : obstacle-aware path selection + path follower
  - far_planner          : global visibility-graph planner

Prerequisites (provided by sim.launch.py + a2_bridge):
  /state_estimation  - ground-truth odometry (published by a2_bridge_sim in a2_unitree_bridge)
  /registered_scan   - world-frame lidar cloud (published by a2_bridge_sim in a2_unitree_bridge)
  /clock             - sim time clock (published by a2_bridge_sim in a2_unitree_bridge)

Usage:
  # Terminal 1
  ros2 launch a2_ros sim.launch.py

  # Terminal 2 (after sim is up)
  ros2 launch a2_ros navigation.launch.py

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
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_dir = get_package_share_directory('a2_description')
    a2_ros_dir      = get_package_share_directory('a2_ros')
    rviz_path       = os.path.join(a2_ros_dir, 'rviz', 'navigation.rviz')
    far_config      = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with navigation config'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    terrain_arg = DeclareLaunchArgument(
        'terrain',
        default_value='indoor',
        description='Terrain type: indoor, open, rough'
    )

    nodes = [
        rviz_arg,
        terrain_arg,
        # Use sim time for all navigation nodes
        SetParameter(name='use_sim_time', value=False),

        # ---- terrain analysis (local map) ----
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize':       0.05,
                'decayTime':           10.0,
                'noDecayDis':          0.0,
                'clearingDis':         8.0,
                'useSorting':          True,
                'quantileZ':           0.25,
                'considerDrop':        True,
                'limitGroundLift':     True,
                'maxGroundLift':       0.25,
                'clearDyObs':          False,
                'minDyObsDis':         0.3,
                'minDyObsAngle':       0.0,
                'minDyObsRelZ':        -0.5,
                'absDyObsRelZThre':    0.2,
                'minDyObsVFOV':        -16.0,
                'maxDyObsVFOV':        16.0,
                'minDyObsPointNum':    1,
                'noDataObstacle':      False,
                'noDataBlockSkipNum':  0,
                'minBlockPointNum':    10,
                'vehicleHeight':       0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'minRelZ':             -1.0,
                'maxRelZ':             1.0,
                'disRatioZ':           0.2,
            }],
        ),

        # ---- terrain analysis ext (global map for far_planner) ----
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[{
                'scanVoxelSize':        0.1,
                'decayTime':            10.0,
                'noDecayDis':           0.0,
                'clearingDis':          30.0,
                'useSorting':           True,
                'quantileZ':            0.25,
                'vehicleHeight':        0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre':  2.0,
                'lowerBoundZ':          -1.0,
                'upperBoundZ':          1.0,
                'disRatioZ':            0.1,
                'checkTerrainConn':     True,
                'terrainUnderVehicle':  -0.75,
                'terrainConnThre':      0.5,
                'ceilingFilteringThre': 2.0,
                'localTerrainMapRadius': 4.0,
            }],
        ),

        # ---- local planner (obstacle avoidance + path following) ----
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('local_planner'),
                    'launch',
                    PythonExpression(["'local_planner_' + '", LaunchConfiguration('terrain'), "' + '.launch'"])
                ])
            ),
            launch_arguments={
                'sensorOffsetX': '0.0',
                'sensorOffsetY': '0.0',
                'vehicleLength': '0.65',
                'vehicleWidth': '0.40',
            }.items()
        ),

        # Node(
        #     package='a2_ros',
        #     executable='nav_vel_relay',
        #     name='nav_vel_relay',
        #     output='screen',
        # ),

        # ---- far_planner (global visibility-graph planner) ----
        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            # Run headless: no X display in container/SSH, so force Qt offscreen
            # to avoid the xcb plugin aborting (SIGABRT). Planning still works;
            # use RViz instead of the FAR Planner GUI for visualization.
            additional_env={'QT_QPA_PLATFORM': 'offscreen'},
            parameters=[far_config],
            remappings=[
                ('/odom_world',         '/state_estimation'),
                ('/terrain_cloud',      '/terrain_map_ext'),
                ('/scan_cloud',         '/registered_scan'),
                ('/terrain_local_cloud','/terrain_map'),
            ],
        ),

        # ---- RViz with navigation config ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    return LaunchDescription(nodes)


def _pkg_exists(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except Exception:
        return False
