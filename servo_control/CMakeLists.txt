cmake_minimum_required(VERSION 3.8)
project(servo_control LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    tf2_geometry_msgs
    trajectory_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

# ##############################################################################
# C++ Libraries
# ##############################################################################

# Auto-generate parameter library from YAML
generate_parameter_library(servo_control_parameters_lib
                           config/servo_parameters.yaml)

# Core C++ library (no ROS communication, no collision detection)
add_library(servo_control_lib_cpp SHARED
    src/servo.cpp
    src/utils/common.cpp
    src/utils/command.cpp)
target_link_libraries(servo_control_lib_cpp servo_control_parameters_lib)
ament_target_dependencies(servo_control_lib_cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS 2 wrapper library
add_library(servo_control_lib_ros SHARED src/servo_node.cpp)
target_link_libraries(servo_control_lib_ros servo_control_lib_cpp)
ament_target_dependencies(servo_control_lib_ros ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Components
# ##############################################################################

rclcpp_components_register_node(servo_control_lib_ros PLUGIN
                                "servo_control::ServoNode" EXECUTABLE servo_node)

# Keyboard input demo executable
add_executable(servo_keyboard_input src/demo/servo_keyboard_input.cpp)
target_link_libraries(servo_keyboard_input servo_control_lib_ros)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Joy controller executable
add_executable(joy_input_control src/demo/joy_input_control.cpp)
target_link_libraries(joy_input_control servo_control_lib_ros)
ament_target_dependencies(joy_input_control ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Install
# ##############################################################################

# Install libraries
install(
  TARGETS servo_control_lib_cpp servo_control_lib_ros servo_control_parameters_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/${PROJECT_NAME})

# Install executable
install(
  TARGETS servo_node servo_keyboard_input joy_input_control
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install include, config directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
