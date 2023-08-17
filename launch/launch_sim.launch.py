import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    x_arg = DeclareLaunchArgument("x", default_value="0", description="x position")
    y_arg = DeclareLaunchArgument("y", default_value="0", description="y position")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="/home/zealzel/zbot2_ws/src/articubot_one/worlds/obstacles.world",
        description="Name of the map",
    )

    package_name = "articubot_one"  # <--- CHANGE ME
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "joystick.launch.py",
                ),
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "my_bot",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
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
            # joystick,
            twist_mux,
            gazebo,
            spawn_entity,
            spawner,
            # diff_drive_spawner,
            # joint_broad_spawner,
        ]
    )
