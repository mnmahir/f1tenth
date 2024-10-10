cmake_minimum_required(VERSION 3.8)
project(ugv_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  nav_msgs
  geometry_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)
endforeach()



###=== CPP MODULES ===###
# Include CPP "include" directory
include_directories(include)

# Create CPP executable
add_executable(cutoff_and_brake_control src/cutoff_and_brake_control.cpp)
ament_target_dependencies(cutoff_and_brake_control rclcpp sensor_msgs std_msgs)

add_executable(cmd_vel_to_ackermann src/cmd_vel_to_ackermann.cpp)
ament_target_dependencies(cmd_vel_to_ackermann rclcpp geometry_msgs ackermann_msgs)

add_executable(modified_cmd_ackermann src/modified_cmd_ackermann.cpp)
ament_target_dependencies(modified_cmd_ackermann rclcpp ackermann_msgs)

# Install CPP executables
install(TARGETS
  cutoff_and_brake_control
  cmd_vel_to_ackermann
  modified_cmd_ackermann
  DESTINATION lib/${PROJECT_NAME})



###=== PYTHON MODULES ===###
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/safety_brake.py
  scripts/cutoff_and_brake_control.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)



###=== TEST ===###
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
