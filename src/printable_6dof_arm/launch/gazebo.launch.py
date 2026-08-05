import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("printable_6dof_arm")
    model = PathJoinSubstitution(
        [package_share, "urdf", "printable_6dof_arm.urdf.xacro"]
    )
    world = PathJoinSubstitution(
        [package_share, "worlds", "color_pick_place.sdf"]
    )
    bridge_config = PathJoinSubstitution(
        [package_share, "config", "gz_bridge.yaml"]
    )
    rviz_config = PathJoinSubstitution([package_share, "config", "arm.rviz"])
    target_color = LaunchConfiguration("target_color")
    robot_description = ParameterValue(
        Command(["xacro", " ", model, " target_color:=", target_color]),
        value_type=str,
    )
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    gazebo_resource_root = PathJoinSubstitution(
        [FindPackagePrefix("printable_6dof_arm"), "share"]
    )
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [
            gazebo_resource_root,
            os.pathsep,
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r -v 3 ",
                IfElseSubstitution(headless, "-s ", ""),
                world,
            ]
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                # Gazebo can publish a joint update with an earlier simulated
                # timestamp while controllers activate.  Accept it instead of
                # intermittently dropping the transform in RViz.
                "ignore_timestamp": True,
                "publish_frequency": 60.0,
            }
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "printable_6dof_arm",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.001",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        arguments=[
            "/world/color_pick_place/set_pose@"
            "ros_gz_interfaces/srv/SetEntityPose"
        ],
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_spawner,
                        arm_spawner,
                        gripper_spawner,
                    ],
                )
            ],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_color",
                default_value="red",
                description="Object whose simulated grasp joint is loaded",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server (no Gazebo GUI)",
            ),
            gazebo_resource_path,
            gazebo,
            robot_state_publisher,
            bridge,
            spawn_robot,
            start_controllers,
            rviz,
        ]
    )
