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
                executable="controller_node",
                name="controller_node",
                namespace=namespace,
                output="screen",
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
