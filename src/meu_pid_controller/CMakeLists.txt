cmake_minimum_required(VERSION 3.8)

project(meu_pid_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(meu_pid_controller SHARED
  src/pid_controller.cpp
)

ament_target_dependencies(meu_pid_controller
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(
  nav2_core
  config/meu_pid_controller.xml
)

install(
  TARGETS meu_pid_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES config/meu_pid_controller.xml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)

ament_package()
