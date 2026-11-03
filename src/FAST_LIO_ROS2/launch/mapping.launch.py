import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='Required. PCD path the map is saved to (an existing '
                    'file is kept and the map saved as <name>_YYYYmmdd_HHMMSS.pcd)'
    )

    def fast_lio_node(context):
        # The save path is required on the command line: a path left in the
        # yaml from another PC fails to save (missing directory) only at the
        # end of mapping.
        map_file_path = LaunchConfiguration('map_file_path').perform(context).strip()
        if not map_file_path:
            raise RuntimeError(
                'map_file_path is required, e.g. '
                'map_file_path:=$HOME/at_home_localization/maps/<name>.pcd')
        map_file_path = os.path.abspath(os.path.expanduser(map_file_path))
        os.makedirs(os.path.dirname(map_file_path), exist_ok=True)
        return [Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[PathJoinSubstitution([config_path, config_file]),
                        {'use_sim_time': use_sim_time,
                         'map_file_path': map_file_path}],
            output='screen'
        )]
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_file_path_cmd)

    ld.add_action(OpaqueFunction(function=fast_lio_node))
    ld.add_action(rviz_node)

    return ld
