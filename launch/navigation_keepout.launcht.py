from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)


def get_path(package_name, subpaths):
    package_share_directory = PathJoinSubstitution(
        [FindPackageShare(package_name)] + subpaths
    )
    return package_share_directory


def generate_launch_description():
    package_name = "articubot_one"

    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    costmap_filter_info_launch_path = get_path(package_name, ["launch", "costmap_filter_info.launch.py"])
    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])
    default_map_path = get_path("turtlebot3_navigation2", ["map", "map.yaml"])
    default_parmas_path = get_path(package_name, ["config", "nav2_params.yaml.20230512"])

    use_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="false", description="Run rviz"
    )
    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_path,
        description="Navigation map path",
    )
    mask_arg = DeclareLaunchArgument(
        "mask",
        description="mask file for keepout layer",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_parmas_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
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
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    costmap_filter_info_delayed = LaunchDescription([
        TimerAction(
            period=4.0,
            actions=[costmap_filter_info],
        )],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
    )
    return LaunchDescription(
        [
            use_sim_arg,
            use_rviz_arg,
            map_arg,
            mask_arg,
            params_arg,
            keepout_params_arg,
            nav2_bringup,
            costmap_filter_info_delayed,
            rviz,
        ]
    )
