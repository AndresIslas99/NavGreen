cmake_minimum_required(VERSION 3.8)
project(agv_rail_approach)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

add_executable(rail_approach_node
  src/main.cpp
  src/rail_approach_node.cpp
)

target_include_directories(rail_approach_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rail_approach_node ${OpenCV_LIBS})

ament_target_dependencies(rail_approach_node
  rclcpp rclcpp_action std_msgs std_srvs geometry_msgs sensor_msgs
  nav_msgs nav2_msgs tf2_ros apriltag_msgs agv_interfaces
)

install(TARGETS rail_approach_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# launch/ exists only on commissioned robots (untracked); OPTIONAL keeps
# fresh clones and CI building.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fine_servo_controller
    test/test_fine_servo_controller.cpp)
  target_include_directories(test_fine_servo_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_fine_servo_controller ${OpenCV_LIBS})
  target_compile_features(test_fine_servo_controller PUBLIC cxx_std_17)
endif()

ament_package()
