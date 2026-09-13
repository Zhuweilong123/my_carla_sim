from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("lightweight_sim") + "/config/default.yaml"
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for one simulator/planner/controller instance",
            ),
            Node(
                package="lightweight_sim",
                executable="simulator_node",
                name="simulator_node",
                namespace=namespace,
                output="screen",
                parameters=[config, {"use_sim_time": False}],
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
        ]
    )
