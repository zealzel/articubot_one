import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def include_launch_description(launch_path, **kwargs):
    launch_args = kwargs.pop("launch_arguments", {})
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_args.items(),
        **kwargs,
    )


def generate_launch_description():
    package_name = "articubot_one"
    MAP_NAME = "turtlebot3_world" # or office_res002_0926
    # simulation only: 2wd|4wd|macanum|zbotlinolong
    params_file_path = get_path(package_name, ["config", "nav2_params.yaml"])
    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file_path,
        description="nav2 params file path",
    )
    rviz_arg = DeclareLaunchArgument(name="rviz", default_value="true")
    map_name_arg = DeclareLaunchArgument("map_name", default_value=MAP_NAME)
    maploc = os.path.join(get_package_share_directory("linorobot2_navigation"), 'maps')
    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=[f'{maploc}/', LaunchConfiguration("map_name"), ".yaml"],
        description="Navigation map path",
    )
    nav2_bringup = include_launch_description(
        nav2_launch_path,
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": True,
            "params_file": LaunchConfiguration("params_file"),
        },
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
            params_file_arg,
            rviz_arg,
            map_name_arg,
            map_arg,
            nav2_bringup,
            rviz,
        ]
    )
