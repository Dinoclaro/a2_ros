"""
Shared CMU autonomy base: terrain analysis + local planner + path follower.

Used by exploration, navigation, and mission launches. Expects:
  /state_estimation  - odometry (DLIO or bridge)
  /registered_scan   - map-frame lidar cloud

Usage (standalone):
  ros2 launch a2_ros autonomy_base.launch.py use_sim_time:=false
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autonomy_speed = LaunchConfiguration('autonomy_speed')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (/clock)',
    )
    autonomy_speed_arg = DeclareLaunchArgument(
        'autonomy_speed',
        default_value='2.0',
        description='Autonomy speed for local planner and path follower',
    )

    local_planner_paths = (
        get_package_share_directory('local_planner') + '/paths'
    )

    sim_time_param = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }

    terrain_analysis = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[sim_time_param, {
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
    )

    terrain_analysis_ext = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[sim_time_param, {
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
    )

    local_planner = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[sim_time_param, {
            'pathFolder':          local_planner_paths,
            'vehicleLength':       0.65,
            'vehicleWidth':        0.40,
            'sensorOffsetX':       0.0,
            'sensorOffsetY':       0.0,
            'twoWayDrive':         False,
            'laserVoxelSize':      0.05,
            'terrainVoxelSize':    0.2,
            'useTerrainAnalysis':  True,
            'checkObstacle':       True,
            'checkRotObstacle':    True,
            'adjacentRange':       3.5,
            'obstacleHeightThre':  0.25,
            'groundHeightThre':    0.1,
            'costHeightThre':      0.1,
            'costScore':           0.02,
            'useCost':             False,
            'pointPerPathThre':    2,
            'minRelZ':             -0.5,
            'maxRelZ':             0.8,
            'maxSpeed':            0.5,
            'dirWeight':           0.1,
            'dirThre':             90.0,
            'dirToVehicle':        False,
            'pathScale':           1.0,
            'minPathScale':        0.75,
            'pathScaleStep':       0.25,
            'pathScaleBySpeed':    True,
            'minPathRange':        1.0,
            'pathRangeStep':       0.5,
            'pathRangeBySpeed':    True,
            'pathCropByGoal':      True,
            'autonomyMode':        True,
            'autonomySpeed':       autonomy_speed,
            'joyToSpeedDelay':     2.0,
            'joyToCheckObstacleDelay': 5.0,
            'goalClearRange':      0.4,
            'goalX':               0.0,
            'goalY':               0.0,
        }],
    )

    path_follower = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[sim_time_param, {
            'sensorOffsetX':    0.0,
            'sensorOffsetY':    0.0,
            'pubSkipNum':       1,
            'twoWayDrive':      False,
            'lookAheadDis':     0.4,
            'yawRateGain':      10.0,
            'stopYawRateGain':  8.0,
            'maxYawRate':       45.0,
            'maxSpeed':         0.5,
            'maxAccel':         2.0,
            'switchTimeThre':   1.0,
            'dirDiffThre':      0.1,
            'stopDisThre':      0.3,
            'slowDwnDisThre':   0.6,
            'useInclRateToSlow': False,
            'inclRateThre':     120.0,
            'slowRate1':        0.25,
            'slowRate2':        0.5,
            'slowTime1':        2.0,
            'slowTime2':        2.0,
            'useInclToStop':    False,
            'inclThre':         45.0,
            'stopTime':         5.0,
            'noRotAtStop':      False,
            'noRotAtGoal':      True,
            'autonomyMode':     True,
            'autonomySpeed':    autonomy_speed,
            'joyToSpeedDelay':  2.0,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        autonomy_speed_arg,
        SetParameter(
            name='use_sim_time',
            value=ParameterValue(use_sim_time, value_type=bool),
        ),
        terrain_analysis,
        terrain_analysis_ext,
        local_planner,
        path_follower,
    ])
