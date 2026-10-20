import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_hector_slam')
    nav2_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # Navigation stack only (planner, controller, behaviours, BT navigator).
    # No map_server/AMCL: Hector SLAM provides /map and map -> odom.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'autostart': 'true',
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2.yaml'),
            description='Full path to the Nav2 parameters file'),
        # nav2_bringup's navigation_launch only uses `namespace` to rewrite the
        # params; the nodes must be pushed into it explicitly.
        # Jazzy: the Create 3 takes geometry_msgs/Twist on cmd_vel_unstamped
        # (/cmd_vel is TwistStamped). The velocity smoother's output is the
        # final command, so redirect only that. The first matching rule wins,
        # so this overrides the smoother's own cmd_vel_smoothed -> cmd_vel.
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel_unstamped',
            description='Topic Nav2 publishes its final velocity command on'),
        GroupAction([
            PushRosNamespace(namespace),
            SetRemap(src='cmd_vel_smoothed', dst=cmd_vel_topic),
            nav2,
        ]),
    ])
