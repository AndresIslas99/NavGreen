cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Mission.msg"
  "msg/RailStartPoint.msg"
  "msg/SafetyStatus.msg"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/UpdateZone.srv"
  "srv/SaveWaypoint.srv"
  "srv/ListMissions.srv"
  "srv/ExecuteMission.srv"
  "srv/RailApproach.srv"
  "srv/ListRailStarts.srv"
  DEPENDENCIES std_msgs
)

ament_package()
