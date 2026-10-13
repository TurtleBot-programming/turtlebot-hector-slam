import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_hector_slam')
    nav2_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

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
        nav2,
    ])
