import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_dir = get_package_share_directory('tb4_hector_slam')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'hector_slam.yaml'),
        description='Full path to the Hector SLAM parameters file')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz alongside SLAM')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'hector_slam.rviz'),
        description='Full path to the RViz config file')

    hector_mapping_node = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        name='hector_mapping',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    trajectory_server_node = Node(
        package='hector_trajectory_server',
        executable='hector_trajectory_server',
        name='hector_trajectory_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(hector_mapping_node)
    ld.add_action(trajectory_server_node)
    ld.add_action(rviz_node)

    return ld
