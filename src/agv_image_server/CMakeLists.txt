cmake_minimum_required(VERSION 3.8)
project(agv_image_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_server_node src/image_server_node.cpp)
ament_target_dependencies(image_server_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_server_node ${OpenCV_LIBS})
target_compile_features(image_server_node PUBLIC cxx_std_17)

install(TARGETS image_server_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image_pipeline test/test_image_pipeline.cpp)
  target_link_libraries(test_image_pipeline ${OpenCV_LIBS})
  target_include_directories(test_image_pipeline PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_compile_features(test_image_pipeline PUBLIC cxx_std_17)
endif()

ament_package()
