cmake_minimum_required(VERSION 3.8)
project(skywarden_controller)

# Only ament_cmake is strictly required to build the package structure
find_package(ament_cmake REQUIRED)

# Install the Python script so 'ros2 run' can find and execute it
install(PROGRAMS scripts/controller_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
