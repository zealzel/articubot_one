from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "articubot_one"

    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])

    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])
    default_map_path = get_path(package_name, ["maps", "turtlebot3_world.yaml"])
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

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_parmas_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
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
            params_arg,
            nav2_bringup,
            rviz,
        ]
    )
