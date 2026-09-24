"""Unified simulator entrypoint with controller mode arbitration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory("lightweight_sim") + "/config/default.yaml"
    gui = LaunchConfiguration("gui")
    scenario = LaunchConfiguration("scenario")
    mode = LaunchConfiguration("mode")
    control_source = LaunchConfiguration("control_source")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true",
                description="Start the unified Pygame mode-control GUI",
            ),
            DeclareLaunchArgument(
                "scenario", default_value="default",
                description="Initial scenario key",
            ),
            DeclareLaunchArgument(
                "mode", default_value="CRUISE",
                description="Initial mode: CRUISE, PARKING or EMERGENCY_STOP",
            ),
            DeclareLaunchArgument(
                "control_source", default_value="AUTO",
                description="Initial control source: AUTO or MANUAL",
            ),
            Node(
                package="lightweight_sim",
                executable="simulator_node",
                name="simulator_node",
                output="screen",
                parameters=[config, {"use_sim_time": False, "scenario": scenario}],
            ),
            Node(
                package="lightweight_sim",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=[config, {"use_sim_time": True}],
            ),
            Node(
                package="lightweight_sim",
                executable="controller_node",
                name="cruise_controller_node",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": True, "output_topic": "control_command/cruise"},
                ],
            ),
            Node(
                package="parking_module",
                executable="parking_controller_node",
                name="parking_controller_node",
                output="screen",
                parameters=[{"use_sim_time": True, "output_topic": "control_command/parking"}],
            ),
            Node(
                package="lightweight_sim",
                executable="controller_manager_node",
                name="controller_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "default_mode": mode,
                        "default_control_source": control_source,
                    }
                ],
            ),
            Node(
                package="lightweight_sim",
                executable="gui_node",
                name="simulator_gui",
                output="screen",
                condition=IfCondition(gui),
                parameters=[
                    config,
                    {
                        "use_sim_time": True,
                        "initial_mode": mode,
                        "initial_control_source": control_source,
                    },
                ],
            ),
        ]
    )
