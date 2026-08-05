from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("printable_6dof_arm")
    vision_config = PathJoinSubstitution(
        [package_share, "config", "hardware_vision.yaml"]
    )
    calibration_file = LaunchConfiguration("calibration_file")
    target_color = LaunchConfiguration("target_color")

    return LaunchDescription(
        [
            DeclareLaunchArgument("calibration_file", default_value=""),
            DeclareLaunchArgument("target_color", default_value="red"),
            Node(
                package="printable_6dof_arm",
                executable="camera_calibration_hw.py",
                name="camera_calibration_hw",
                parameters=[
                    vision_config,
                    {"calibration_file": calibration_file},
                ],
                output="screen",
            ),
            Node(
                package="printable_6dof_arm",
                executable="color_tracker_hw_test.py",
                name="color_tracker_hw_test",
                parameters=[
                    vision_config,
                    {"target_color": target_color},
                ],
                output="screen",
            ),
            Node(
                package="printable_6dof_arm",
                executable="pixel_to_tabletop_hw.py",
                name="pixel_to_tabletop_hw",
                parameters=[vision_config],
                output="screen",
            ),
        ]
    )
