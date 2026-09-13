from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("lightweight_sim") + "/config/default.yaml"
    return LaunchDescription(
        [
            Node(
                package="lightweight_sim",
                executable="simulator_node",
                name="simulator_node",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="lightweight_sim",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="lightweight_sim",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
