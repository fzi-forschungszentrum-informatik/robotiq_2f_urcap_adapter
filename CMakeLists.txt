cmake_minimum_required(VERSION 3.5)
project(robotiq_2f85_urcap_adapter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveGripper.action"
)

# Install Python executables
install(PROGRAMS
  scripts/robotiq_2f85_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
