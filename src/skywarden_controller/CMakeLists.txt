cmake_minimum_required(VERSION 3.8)
project(skywarden_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(skywarden_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node skywarden_interfaces std_msgs rclcpp)

install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})

ament_package()
