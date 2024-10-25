cmake_minimum_required(VERSION 3.8)
project(ugv_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros
  tf2_geometry_msgs
)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)
# Create CPP executable
add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2_geometry_msgs)

add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)
ament_target_dependencies(waypoint_visualizer rclcpp geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs) 

add_executable(reactive_node src/reactive_node.cpp)
ament_target_dependencies(reactive_node rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs)

add_executable(avoidance_controller src/avoidance_controller.cpp)
ament_target_dependencies(avoidance_controller rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2_geometry_msgs)

# Install CPP executables
install(TARGETS
  pure_pursuit
  waypoint_visualizer
  reactive_node
  avoidance_controller
  DESTINATION lib/${PROJECT_NAME})



###=== PYTHON MODULES ===###
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  # scripts/safety_brake.py
  # scripts/cutoff_and_brake_control.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  config
  launch
  maps
  racelines
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
