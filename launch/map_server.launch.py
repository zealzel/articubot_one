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
    costmap_filter_info_launch_path = get_path(
        package_name, ["launch", "costmap_filter_info.launch.py"]
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

    # if not delayed, nav2_bringup will not able to launch controllers successfully
    costmap_filter_info_delayed = LaunchDescription(
        [
            TimerAction(
                period=4.0,
                actions=[costmap_filter_info],
            )
        ],
    )

    return LaunchDescription(
        [
            keepout_params_arg,
            map_arg,
            map_sim_arg,
            mask_arg,
            mask_sim_arg,
            map_server,
            map_server_lifecyle,
            costmap_filter_info_delayed,
        ]
    )
