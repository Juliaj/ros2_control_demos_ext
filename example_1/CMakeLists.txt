cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_ext_example_1 LANGUAGES C CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib

  rclcpp
  rclcpp_lifecycle
  rclcpp_action

  std_msgs
  geometry_msgs
  control_msgs
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# compile 
add_library(
  ros2_control_demo_ext_example_1 
  SHARED 
  hardware/ackermannbot_system.cpp
)

target_compile_features(ros2_control_demo_ext_example_1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ros2_control_demo_ext_example_1 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  ros2_control_demo_ext_example_1
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# export hardware plugins
# pluginlib_export_plugin_description_files(hardware_interface, ros2_control_demo_ext_example_1.xml)

# install 
install (
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_ext_example_1
)

install(
  DIRECTORY description/gazebo description/launch description/ros2_control description/urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ros2_control_demo_ext_example_1
  EXPORT export_ros2_control_demo_ext_example_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(example_1_sdf_test test/example_1_sdf_test.py)
endif()


# exports
# ament_export_targets(export_ros2_control_demo_ext_example_1 HAS_LIBRARY_TARGETS)
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
