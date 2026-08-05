from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("printable_6dof_arm")
    model = PathJoinSubstitution(
        [package_share, "urdf", "printable_6dof_arm.urdf.xacro"]
    )
    color_config = PathJoinSubstitution(
        [package_share, "config", "color_tracker.yaml"]
    )
    servo_config = PathJoinSubstitution(
        [package_share, "config", "servo_calibration.yaml"]
    )
    rviz_config = PathJoinSubstitution([package_share, "config", "arm.rviz"])
    robot_description = ParameterValue(
        Command(["xacro", " ", model]), value_type=str
    )

    dry_run = LaunchConfiguration("dry_run")
    target_color = LaunchConfiguration("target_color")
    use_rviz = LaunchConfiguration("use_rviz")
    auto_start = LaunchConfiguration("auto_start")
    dashboard_port = LaunchConfiguration("dashboard_port")
    open_dashboard = LaunchConfiguration("open_dashboard")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("target_color", default_value="red"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("dashboard_port", default_value="8080"),
            DeclareLaunchArgument("open_dashboard", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", "/arm_hardware/joint_states")],
                output="screen",
            ),
            Node(
                package="printable_6dof_arm",
                executable="servo_bridge.py",
                name="servo_bridge",
                parameters=[
                    servo_config,
                    {"dry_run": ParameterValue(dry_run, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="printable_6dof_arm",
                executable="color_tracker.py",
                name="color_tracker",
                parameters=[color_config, {"target_color": target_color}],
                output="screen",
            ),
            Node(
                package="printable_6dof_arm",
                executable="pick_place_controller.py",
                name="pick_place_controller",
                parameters=[
                    color_config,
                    {
                        "arm_command_topic": "/arm_hardware/joint_trajectory",
                        "gripper_command_topic": "/arm_hardware/gripper_trajectory",
                        "target_color": target_color,
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                        "use_sim_grasp_assist": False,
                        # One physical servo mechanically mirrors both jaws.
                        "gripper_joint_names": ["left_finger_joint"],
                        "gripper_joint_multipliers": [-1.0],
                    },
                ],
                output="screen",
            ),
            Node(
                package="printable_6dof_arm",
                executable="arm_dashboard.py",
                name="arm_dashboard",
                parameters=[
                    {
                        "target_color": target_color,
                        "port": ParameterValue(dashboard_port, value_type=int),
                        "open_browser": ParameterValue(
                            open_dashboard, value_type=bool
                        ),
                        "joint_state_topic": "/arm_hardware/joint_states",
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
