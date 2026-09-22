cmake_minimum_required(VERSION 3.8)
project(lightweight_sim_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/PathPoint.msg"
  "msg/Path.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/ControlCommand.msg"
  "msg/ControlMode.msg"
  "msg/SimulationStatus.msg"
  "msg/RouteRequest.msg"
  "msg/RouteSegment.msg"
  "msg/RoutePlan.msg"
  "msg/ReferenceLine.msg"
  "srv/ComputeRoute.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
