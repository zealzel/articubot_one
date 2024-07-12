import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "articubot_one"

    default_map_path = get_path("fitrobot", ["maps", "office_res002_0523.yaml"])
    default_map_path_sim = get_path(package_name, ["maps", "turtlebot3_world.yaml"])

    default_mask_path = get_path(
        "fitrobot", ["masks", "keepout_mask_office_res002_0523.yaml"]
    )
    default_mask_path_sim = get_path(
        package_name, ["masks", "keepout_mask_turtlebot3_world.yaml"]
    )
    default_params_file_path = get_path(
        package_name, ["config", "nav2_params_keepout_multi.yaml"]
    )
    costmap_filter_info_launch_path = get_path(
        package_name, ["launch", "costmap_filter_info.launch.py"]
    )

    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="Namespace",
    )
    use_namespace_arg = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="false", description="Run rviz"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=get_path(package_name, ["rviz", "multi_nav2_default_view.rviz"]),
        description=("Full path to the ROS2 rviz config file"),
    )

    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_path,
        description="Navigation map path",
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )
    map_sim_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_path_sim,
        description="Navigation map path",
        condition=IfCondition(LaunchConfiguration("sim")),
    )
    mask_arg = DeclareLaunchArgument(
        "mask",
        default_value=default_mask_path,
        description="mask file for keepout layer",
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )
    mask_sim_arg = DeclareLaunchArgument(
        "mask",
        default_value=default_mask_path_sim,
        description="mask file for keepout layer",
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file_path,
        description=(
            "Full path to the ROS2 parameters file to use for all launched nodes"
        ),
    )
    keepout_params_arg = DeclareLaunchArgument(
        "keepout_params_file",
        default_value=get_path(package_name, ["params", "keepout_params.yaml"]),
        description="params file for keepout layer",
    )

    costmap_filter_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(costmap_filter_info_launch_path),
        launch_arguments={
            "params_file": LaunchConfiguration("keepout_params_file"),
            "mask": LaunchConfiguration("mask"),
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": default_map_path_sim,
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    map_server_lifecyle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim")},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    nav_launch_dir = os.path.join(
        get_package_share_directory(package_name), "launch", "nav2_bringup"
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "map": "",
            "map_server": "False",
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    # if not delayed, nav2_bringup will not able to launch controllers successfully
    costmap_filter_info_delayed = LaunchDescription(
        [
            TimerAction(
                period=4.0,
                actions=[costmap_filter_info],
            )
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, "rviz_launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("sim"),
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "log_level": "warn",
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            namespace_arg,
            use_namespace_arg,
            use_sim_arg,
            use_rviz_arg,
            rviz_config_arg,
            keepout_params_arg,
            map_arg,
            map_sim_arg,
            mask_arg,
            mask_sim_arg,
            map_server,
            map_server_lifecyle,
            params_arg,
            nav2_bringup,
            costmap_filter_info_delayed,
            rviz,
        ]
    )
