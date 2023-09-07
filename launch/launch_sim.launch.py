import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "articubot_one"

    rsp_launch_path = get_path(package_name, ["launch", "rsp.launch.py"])
    gazebo_launch_path = get_path("gazebo_ros", ["launch", "gazebo.launch.py"])
    world_path = get_path(package_name, ["worlds", "obstacles.world"])

    x_arg = DeclareLaunchArgument("x", default_value="0", description="x position")
    y_arg = DeclareLaunchArgument("y", default_value="0", description="y position")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Name of the map",
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )
    twist_mux_params = get_path(package_name, ["config", "twist_mux.yaml"])
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), "config", "gazebo_params.yaml")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_bot",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
        ],
        output="screen",
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager-timeout", "10"],
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    spawner = LaunchDescription([
        TimerAction(
            period=4.0,
            actions=[diff_drive_spawner, joint_broad_spawner],
        )],
    )
    return LaunchDescription(
        [
            world_arg,
            x_arg,
            y_arg,
            rsp,
            twist_mux,
            gazebo,
            spawn_entity,
            spawner,
        ]
    )
