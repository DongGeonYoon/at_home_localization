"""Bring up localization, the structural map, and the static/local costmaps.

Scope ends at the costmaps and TF the MPPI team consumes; no controller,
planner, goal handling or /cmd_vel is started here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def repo_maps_dir():
    """<repo>/maps of the checkout this file was built from.

    Walks up from this file (src/ with --symlink-install, else install/ or
    build/), so the default works wherever the repository was cloned.
    """
    path = os.path.dirname(os.path.realpath(__file__))
    while path != os.path.dirname(path):
        if all(os.path.isdir(os.path.join(path, d)) for d in ("maps", "src")):
            return os.path.join(path, "maps")
        path = os.path.dirname(path)
    return os.path.expanduser("~/at_home_localization/maps")


def costmap_nodes(name, config, condition, use_sim_time, overrides=None):
    """Costmap host plus its lifecycle manager; the node FQN is /<name>/<name>."""
    host = Node(
        package="costmap_host",
        executable="costmap_host",
        arguments=[name],
        output="screen",
        parameters=[config, {"use_sim_time": use_sim_time, **(overrides or {})}],
        condition=condition,
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=f"lifecycle_manager_{name}",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [f"/{name}/{name}"],
            # A standalone Costmap2DROS does not create a Nav2 bond.
            "bond_timeout": 0.0,
            "use_sim_time": use_sim_time,
        }],
        condition=condition,
    )
    return [host, lifecycle]


def zenoh_router(context):
    """rmw_zenoh router; optionally bridged to a remote router (e.g. MPPI PC)."""
    actions = []
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp":
        actions.append(LogInfo(msg=(
            "WARNING: RMW_IMPLEMENTATION is "
            f"'{os.environ.get('RMW_IMPLEMENTATION', '(unset)')}', not "
            "'rmw_zenoh_cpp'; these nodes will not talk to the Zenoh system."
        )))
    if LaunchConfiguration("run_zenoh").perform(context).lower() != "true":
        return actions
    endpoint = LaunchConfiguration("zenoh_endpoint").perform(context).strip()
    env = {}
    if endpoint:
        env["ZENOH_CONFIG_OVERRIDE"] = f'connect/endpoints=["{endpoint}"]'
    actions.append(Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="rmw_zenohd",
        output="screen",
        additional_env=env,
    ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("mppi_nav")
    localization_share = get_package_share_directory("fast_lio_localization")
    livox_share = get_package_share_directory("livox_ros_driver2")
    default_maps = repo_maps_dir()

    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument("run_livox", default_value="false"),
        DeclareLaunchArgument("run_localization", default_value="true"),
        DeclareLaunchArgument("run_map_server", default_value="true"),
        DeclareLaunchArgument("run_static_costmap", default_value="true"),
        DeclareLaunchArgument("run_local_costmap", default_value="true"),
        DeclareLaunchArgument("run_rviz", default_value="false"),
        # Robot-body (self-return) removal in front of the local costmap.
        DeclareLaunchArgument("run_local_cloud_filter", default_value="true"),
        DeclareLaunchArgument(
            "local_cloud_input", default_value="/cloud_registered_body"
        ),
        DeclareLaunchArgument(
            "local_cloud_output", default_value="/local_obstacle_cloud"
        ),
        # Exclusion box in base_link [m]: measured base +-0.27 x +-0.25, LiDAR to
        # x = 0.35, top structure 0.93 m, plus ~5 cm margin.
        DeclareLaunchArgument("self_filter_min_x", default_value="-0.32"),
        DeclareLaunchArgument("self_filter_max_x", default_value="0.40"),
        DeclareLaunchArgument("self_filter_min_y", default_value="-0.30"),
        DeclareLaunchArgument("self_filter_max_y", default_value="0.30"),
        DeclareLaunchArgument("self_filter_min_z", default_value="-0.05"),
        DeclareLaunchArgument("self_filter_max_z", default_value="1.00"),
        # Local Zenoh router (listens on tcp/[::]:7447). Disable when a router
        # already runs on this PC, e.g. started separately or by another team.
        DeclareLaunchArgument("run_zenoh", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "pcd_map", default_value=os.path.join(default_maps, "406.pcd")
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=os.path.join(default_maps, "406_new.yaml"),
        ),
        DeclareLaunchArgument("localization_config", default_value="mid360.yaml"),
        # Remote router to connect to, e.g. tcp/192.168.0.93:7447. Empty keeps
        # the router local; the remote side may also connect to this PC:7447.
        DeclareLaunchArgument("zenoh_endpoint", default_value=""),
        # base_link = base center on the floor, expressed in the Mid-360 IMU
        # (body) frame. Measured: LiDAR 0.31 m ahead of center, 0.445 m high;
        # the IMU is (+0.011, +0.023, -0.044) m from the LiDAR origin.
        # Rotation assumes a level, forward-facing LiDAR (not calibrated).
        DeclareLaunchArgument("base_link_x", default_value="-0.321"),
        DeclareLaunchArgument("base_link_y", default_value="-0.023"),
        DeclareLaunchArgument("base_link_z", default_value="-0.401"),
        DeclareLaunchArgument("base_link_roll", default_value="0.0"),
        DeclareLaunchArgument("base_link_pitch", default_value="0.0"),
        DeclareLaunchArgument("base_link_yaw", default_value="0.0"),
    ]

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360s_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("run_livox")),
    )

    # Publishes map -> camera_init (ICP correction) and camera_init -> body.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("pcd_map"),
            "config_file": LaunchConfiguration("localization_config"),
            "rviz": "false",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_localization")),
    )

    body_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link",
        arguments=[
            "--x", LaunchConfiguration("base_link_x"),
            "--y", LaunchConfiguration("base_link_y"),
            "--z", LaunchConfiguration("base_link_z"),
            "--roll", LaunchConfiguration("base_link_roll"),
            "--pitch", LaunchConfiguration("base_link_pitch"),
            "--yaw", LaunchConfiguration("base_link_yaw"),
            "--frame-id", "body",
            "--child-frame-id", "base_link",
        ],
        output="screen",
    )

    run_map_server = IfCondition(LaunchConfiguration("run_map_server"))
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": LaunchConfiguration("map_yaml"),
            "topic_name": "map",
            "frame_id": "map",
            "use_sim_time": use_sim_time,
        }],
        condition=run_map_server,
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
            "use_sim_time": use_sim_time,
        }],
        condition=run_map_server,
    )

    static_costmap = costmap_nodes(
        "mppi_static_costmap",
        os.path.join(share, "config", "static_costmap.yaml"),
        IfCondition(LaunchConfiguration("run_static_costmap")),
        use_sim_time,
    )
    run_filter = LaunchConfiguration("run_local_cloud_filter")
    self_filter = Node(
        package="mppi_nav",
        executable="self_filter",
        output="screen",
        parameters=[{
            "input_topic": LaunchConfiguration("local_cloud_input"),
            "output_topic": LaunchConfiguration("local_cloud_output"),
            "robot_frame": "base_link",
            **{
                f"self_filter_{k}": PythonExpression(
                    [LaunchConfiguration(f"self_filter_{k}")]
                )
                for k in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")
            },
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", run_filter, "' == 'true' and '",
                LaunchConfiguration("run_local_costmap"), "' == 'true'",
            ])
        ),
    )
    # Mark from the filtered cloud, or from the raw cloud when the filter is off.
    marking_topic = PythonExpression([
        "'", LaunchConfiguration("local_cloud_output"), "' if '", run_filter,
        "' == 'true' else '", LaunchConfiguration("local_cloud_input"), "'",
    ])
    local_costmap = costmap_nodes(
        "local_costmap",
        os.path.join(share, "config", "local_costmap.yaml"),
        IfCondition(LaunchConfiguration("run_local_costmap")),
        use_sim_time,
        overrides={"voxel_layer.lidar.topic": marking_topic},
    )

    clear_on_relocalize = Node(
        package="mppi_nav",
        executable="clear_on_relocalize",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("run_local_costmap")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="costmap_rviz",
        output="screen",
        arguments=["-d", os.path.join(share, "rviz", "costmap_bringup.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("run_rviz")),
    )

    return LaunchDescription(
        arguments
        + [OpaqueFunction(function=zenoh_router)]
        + [livox, localization, body_to_base_link, map_server, map_lifecycle]
        + static_costmap
        + [self_filter]
        + local_costmap
        + [clear_on_relocalize, rviz]
    )
