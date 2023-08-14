import os

from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit


def generate_launch_description():
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
                )
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

    gzserver = ExecuteProcess(
        # cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args', '--params-file', gazebo_params_file],
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            LaunchConfiguration("world"),
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(cmd=["gzclient", "--verbose"], output="screen")

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver,
            on_start=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver,
            on_start=[joint_broad_spawner],
        )
    )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[joint_broad_spawner],
    #     )
    # )

    return LaunchDescription(
        [
            world_arg,
            rsp,
            joystick,
            twist_mux,
            # gazebo,
            gzserver,
            gzclient,
            spawn_entity,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            # diff_drive_spawner,
            # joint_broad_spawner
        ]
    )
