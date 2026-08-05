from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("printable_6dof_arm")
    target_color = LaunchConfiguration("target_color")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    auto_start = LaunchConfiguration("auto_start")
    dashboard_port = LaunchConfiguration("dashboard_port")
    open_dashboard = LaunchConfiguration("open_dashboard")
    color_config = PathJoinSubstitution(
        [package_share, "config", "color_tracker.yaml"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "target_color": target_color,
            "use_rviz": use_rviz,
            "headless": headless,
        }.items(),
    )

    tracker = Node(
        package="printable_6dof_arm",
        executable="color_tracker.py",
        name="color_tracker",
        parameters=[
            color_config,
            {
                "target_color": target_color,
                "use_sim_time": True,
            },
        ],
        output="screen",
    )

    dashboard = Node(
        package="printable_6dof_arm",
        executable="arm_dashboard.py",
        name="arm_dashboard",
        parameters=[
            {
                "target_color": target_color,
                "port": ParameterValue(dashboard_port, value_type=int),
                "open_browser": ParameterValue(open_dashboard, value_type=bool),
            }
        ],
        output="screen",
    )

    pick_place = Node(
        package="printable_6dof_arm",
        executable="pick_place_controller.py",
        name="pick_place_controller",
        parameters=[
            color_config,
            {
                "target_color": target_color,
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "use_sim_time": True,
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_color",
                default_value="red",
                description="HSV target: red, green, or blue",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Start the red-cube pick/place sequence after simulation is ready",
            ),
            DeclareLaunchArgument("dashboard_port", default_value="8080"),
            # Keep the complete simulation experience available from the
            # standard one-command launch alongside Gazebo and RViz.
            DeclareLaunchArgument("open_dashboard", default_value="true"),
            simulation,
            TimerAction(period=7.0, actions=[tracker, pick_place, dashboard]),
        ]
    )
