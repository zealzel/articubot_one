from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "articubot_one"

    map_name_arg = DeclareLaunchArgument(
        "map_name", default_value="fit_office", description="Name of the map"
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false", description="Headless mode"
    )
    map_defaultpos = {
        "tb3": [10, -8, 0.05],
        "fit_office": [7, -17, 2],
        "factory_bk": [30, -22, 0.05],
    }

    rsp_launch_path = get_path(package_name, ["launch", "rsp.launch.py"])

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

    included_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("my_rmf"), "launch", "my_sim.launch.xml"]
            )
        ),
        launch_arguments={
            "map_name": LaunchConfiguration("map_name"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    spawn_robot_node_tb3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x", str(map_defaultpos["tb3"][0]),
            "-y", str(map_defaultpos["tb3"][1]),
            "-z", str(map_defaultpos["tb3"][2]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'tb3'"]))
    )

    spawn_robot_node_fit_office = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x", str(map_defaultpos["fit_office"][0]),
            "-y", str(map_defaultpos["fit_office"][1]),
            "-z", str(map_defaultpos["fit_office"][2]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'fit_office'"]))
    )

    spawn_robot_node_factory_bk = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x", str(map_defaultpos["factory_bk"][0]),
            "-y", str(map_defaultpos["factory_bk"][1]),
            "-z", str(map_defaultpos["factory_bk"][2]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'factory_bk'"]))
    )

    return LaunchDescription(
        [
            map_name_arg,
            headless_arg,
            included_launch,
            rsp,
            twist_mux,
            spawn_robot_node_tb3,
            spawn_robot_node_fit_office,
            spawn_robot_node_factory_bk,
            spawner,
        ]
    )
