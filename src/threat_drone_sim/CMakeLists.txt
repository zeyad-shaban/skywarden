cmake_minimum_required(VERSION 3.8)

project(threat_drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/drone_mover_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drone_mover_node)

install(
  PROGRAMS scripts/camera_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_recorder_node)

ament_package()
