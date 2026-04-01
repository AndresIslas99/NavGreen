cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Mission.msg"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/UpdateZone.srv"
  "srv/SaveWaypoint.srv"
  "srv/ListMissions.srv"
  "srv/ExecuteMission.srv"
)

ament_package()
