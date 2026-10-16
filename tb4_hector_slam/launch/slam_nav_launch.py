import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_hector_slam')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'hector_slam_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'nav2_params_file': nav2_params_file,
        }.items())

    explorer = Node(
        package='tb4_hector_slam',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(LaunchConfiguration('explore')))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2.yaml'),
            description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='Start the frontier explorer (autonomous mapping)'),
        slam,
        nav,
        explorer,
    ])
