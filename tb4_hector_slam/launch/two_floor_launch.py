"""Multi-floor mapping and navigation: one Hector SLAM instance per floor,
the floor manager, Nav2, the frontier explorer (optional) and RViz.

  ros2 launch tb4_hector_slam two_floor_launch.py use_sim_time:=true explore:=true

Per floor N, hector_mapping runs in namespace floorN on /floorN/scan and
publishes /floorN/map. The floor manager feeds it, relays the current floor's
map to /map and owns map -> odom (see scripts/floor_manager.py). The Nav2
costmaps read /scan_flat, which carries no scans taken on a ramp.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_hector_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    floors_file = LaunchConfiguration('floors_file')
    hector_params = LaunchConfiguration('hector_params_file')
    log_level = LaunchConfiguration('log_level')
    floors = 2

    def hector(floor):
        ns = f'floor{floor}'
        params = ParameterFile(
            RewrittenYaml(source_file=hector_params, root_key=ns,
                          param_rewrites={'use_sim_time': use_sim_time},
                          convert_types=True),
            allow_substs=True)
        return Node(
            package='hector_mapping',
            executable='hector_mapping_node',
            name='hector_mapping',
            namespace=ns,
            output='screen',
            parameters=[params, {
                'scan_topic': 'scan',            # -> /floorN/scan
                'map_frame': 'map',              # all floors share one frame
                'pub_map_odom_transform': False,   # the floor manager owns it
                'pub_map_scanmatch_transform': False,
                # start each match from the TF pose (last correction +
                # odometry): carries the pose across ramps and into a new
                # floor's map
                'use_tf_pose_start_estimate': True,
            }],
            arguments=['--ros-args', '--log-level', log_level])

    shared_params = ParameterFile(
        RewrittenYaml(source_file=hector_params, root_key='',
                      param_rewrites={'use_sim_time': use_sim_time},
                      convert_types=True),
        allow_substs=True)

    trajectory_server = Node(
        package='hector_trajectory_server',
        executable='hector_trajectory_server',
        name='hector_trajectory_server',
        output='screen',
        parameters=[shared_params])

    floor_manager = Node(
        package='tb4_hector_slam',
        executable='floor_manager.py',
        name='floor_manager',
        output='screen',
        parameters=[floors_file, {'use_sim_time': use_sim_time, 'floors': floors}])

    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_dir, 'config', 'nav2.yaml'),
        param_rewrites={'topic': '/scan_flat'},   # costmap observation sources
        convert_types=True)

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'nav2_params_file': nav2_params,
        }.items())

    explorer = Node(
        package='tb4_hector_slam',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[floors_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('explore')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='Start the frontier explorer (autonomous multi-floor mapping)'),
        DeclareLaunchArgument(
            'floors_file',
            default_value=os.path.join(pkg_dir, 'config', 'two_floor.yaml'),
            description='Floor manager settings and ramp transitions'),
        DeclareLaunchArgument(
            'hector_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'hector_slam.yaml')),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_dir, 'rviz', 'hector_slam.rviz')),
        DeclareLaunchArgument('log_level', default_value='info'),
        *[hector(f) for f in range(1, floors + 1)],
        trajectory_server,
        floor_manager,
        nav,
        explorer,
        rviz,
    ])
