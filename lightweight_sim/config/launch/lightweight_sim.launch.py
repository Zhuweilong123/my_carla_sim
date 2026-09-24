from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("lightweight_sim") + "/config/default.yaml"
    namespace = LaunchConfiguration("namespace")
    gui = LaunchConfiguration("gui")
    controller_enabled = LaunchConfiguration("controller_enabled")
    routing_enabled = LaunchConfiguration("routing_enabled")
    reference_line_enabled = LaunchConfiguration("reference_line_enabled")
    scenario = LaunchConfiguration("scenario")
    steering = LaunchConfiguration("steering_profile")
    return LaunchDescription(
        [
            DeclareLaunchArgument("steering_profile", default_value="ideal",
                description="ideal or assumed (uncalibrated steering delay/lag/rate limits)"),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for one simulator/planner/controller instance",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start the ROS 2 Pygame visualization client",
            ),
            DeclareLaunchArgument(
                "controller_enabled",
                default_value="true",
                description="Start the built-in forward tracking controller",
            ),
            DeclareLaunchArgument(
                "routing_enabled",
                default_value="true",
                description="Start the independent lane-level A* routing node",
            ),
            DeclareLaunchArgument(
                "reference_line_enabled",
                default_value="true",
                description="Generate map-derived reference lines from Routing output",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="obstacle",
                description="Initial scenario key",
            ),
            Node(
                package="lightweight_sim",
                executable="simulator_node",
                name="simulator_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": False, "scenario": scenario, "steering_profile": steering},
                ],
            ),
            Node(
                package="lightweight_sim",
                executable="planner_node",
                name="planner_node",
                namespace=namespace,
                output="screen",
                parameters=[config, {"use_sim_time": True}],
            ),
            Node(
                package="lightweight_sim",
                executable="routing_node",
                name="routing_node",
                namespace=namespace,
                output="screen",
                condition=IfCondition(routing_enabled),
                parameters=[config, {"use_sim_time": True}],
            ),
            Node(
                package="lightweight_sim",
                executable="reference_line_node",
                name="reference_line_node",
                namespace=namespace,
                output="screen",
                condition=IfCondition(reference_line_enabled),
                parameters=[config, {"use_sim_time": True}],
            ),
            Node(
                package="lightweight_sim",
                executable="controller_node",
                name="controller_node",
                namespace=namespace,
                output="screen",
                condition=IfCondition(controller_enabled),
                parameters=[config, {"use_sim_time": True}],
            ),
            Node(
                package="lightweight_sim",
                executable="gui_node",
                name="simulator_gui",
                namespace=namespace,
                output="screen",
                condition=IfCondition(gui),
                parameters=[config, {"use_sim_time": True}],
            ),
        ]
    )
