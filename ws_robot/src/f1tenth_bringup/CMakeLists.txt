cmake_minimum_required(VERSION 3.8)
project(f1tenth_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(PACKAGE std_msgs sensor_msgs nav_msgs geometry_msgs rcl_interfaces)
  find_package(${PACKAGE} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Telemetry.msg
  msg/DriveState.msg
  msg/LapState.msg
  msg/ComponentState.msg
  msg/SupervisorState.msg
  msg/RecorderState.msg
  msg/LocalizationState.msg
  srv/SetMode.srv
  srv/SaveFile.srv
  srv/FileOp.srv
  srv/ComponentCommand.srv
  srv/GetMapPreview.srv
  srv/GetPath.srv
  srv/OptimizeRaceline.srv
  srv/GetConsole.srv
  DEPENDENCIES std_msgs sensor_msgs nav_msgs geometry_msgs rcl_interfaces
)

# The UI needs Qt5 and RViz's development files; without them only the interfaces and launch files are built
option(BUILD_UI "Build the f1tenth_ui driver UI" ON)
if(BUILD_UI)
  find_package(Qt5 QUIET COMPONENTS Core Gui Widgets Network)
  find_package(rviz_common QUIET)
  find_package(rviz_rendering QUIET)
  find_package(rviz_default_plugins QUIET)
  if(NOT Qt5_FOUND OR NOT rviz_common_FOUND OR NOT rviz_rendering_FOUND OR NOT rviz_default_plugins_FOUND)
    message(WARNING "Qt5 or RViz development files not found: f1tenth_ui will not be built")
    set(BUILD_UI OFF)
  endif()
endif()
if(BUILD_UI)
  foreach(PACKAGE rclcpp diagnostic_msgs std_srvs ament_index_cpp)
    find_package(${PACKAGE} REQUIRED)
  endforeach()
  file(GLOB UI_SOURCES CONFIGURE_DEPENDS src/ui/*.cpp src/ui/*.hpp)
  add_executable(f1tenth_ui ${UI_SOURCES} resources/ui.qrc)
  set_target_properties(f1tenth_ui PROPERTIES AUTOMOC ON AUTORCC ON)
  target_include_directories(f1tenth_ui PRIVATE src/ui)
  ament_target_dependencies(f1tenth_ui rclcpp rcl_interfaces diagnostic_msgs std_msgs std_srvs sensor_msgs nav_msgs
    geometry_msgs rviz_common rviz_rendering rviz_default_plugins ament_index_cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(f1tenth_ui Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network ${cpp_typesupport_target})
  install(TARGETS f1tenth_ui DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
