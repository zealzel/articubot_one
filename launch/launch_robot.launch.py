from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "articubot_one"

    rsp_launch_path = get_path(package_name, ["launch", "rsp.launch.py"])
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={"use_sim_time": "false", "use_ros2_control": "true"}.items(),
    )

    twist_mux_params = get_path(package_name, ["config", "twist_mux.yaml"])
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    controller_params_file = get_path(package_name, ["config", "my_controllers.yaml"])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params_file],
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    rplidar_node = Node(
        name="rplidar_component",
        package="rplidar_ros",
        executable="rplidar_composition",
        parameters=[
            {
                # "serial_port": "/dev/ttyUSB1",
                "serial_port": "/dev/rplidar",
                "serial_baudrate": 115200,
                "frame_id": "laser_frame",
                "angle_compensate": True,
                "inverted": False,
                "scan_mode": "Standard",
            }
        ],
    )
    return LaunchDescription(
        [
            rsp,
            twist_mux,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            rplidar_node,
        ]
    )
