from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("printable_6dof_arm")
    model = PathJoinSubstitution(
        [package_share, "urdf", "printable_6dof_arm.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution([package_share, "config", "arm.rviz"])

    use_gui = LaunchConfiguration("use_gui")
    robot_description = ParameterValue(
        Command(["xacro", " ", model]), value_type=str
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(use_gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=UnlessCondition(use_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
