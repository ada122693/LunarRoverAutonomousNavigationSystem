cmake_minimum_required(VERSION 3.16)
project(haphazard)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(path_schedule_plan_server
  haphazard.cpp
)

target_include_directories(path_schedule_plan_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

ament_target_dependencies(path_schedule_plan_server
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  path_schedule_plan_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
