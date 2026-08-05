from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("printable_6dof_arm")
    calibration = PathJoinSubstitution(
        [package_share, "config", "servo_calibration.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Keep true until mechanical zeros and limits are verified.",
            ),
            Node(
                package="printable_6dof_arm",
                executable="servo_bridge.py",
                name="servo_bridge",
                parameters=[
                    calibration,
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        )
                    },
                ],
                output="screen",
            ),
        ]
    )
