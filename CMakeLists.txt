cmake_minimum_required(VERSION 3.5)
project(robotiq_2f85_urcap_adapter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveGripper.action"
)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/urcap_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
