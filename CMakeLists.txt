cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_urcap_adapter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveGripper.action"
  "action/GripperCommand.action"
  "msg/GripperCommand.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME}_socket)

# Install Python executables
install(PROGRAMS
  scripts/robotiq_2f_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
