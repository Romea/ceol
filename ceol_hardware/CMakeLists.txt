cmake_minimum_required(VERSION 3.10)
project(ceol_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pal_statistics REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_socketcan)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(romea_mobile_base_hardware REQUIRED)
find_package(romea_implement_msgs)


## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ceol_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC
  ROS_DISTRO=${ROS_DISTRO})

target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${ros2_socketcan_LIBRARIES}
  romea_core_common::romea_core_common
  romea_core_mobile_base::romea_core_mobile_base
  romea_common_utils::romea_common_utils
  romea_mobile_base_utils::romea_mobile_base_utils
  romea_mobile_base_hardware::romea_mobile_base_hardware
  ${romea_implement_msgs_TARGETS}
)

# These dependencies still use legacy CMake exports.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${ros2_socketcan_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  ${ros2_socketcan_DEFINITIONS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface ceol_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

##if(BUILD_TESTING)
##  find_package(ament_cmake_gtest REQUIRED)
##endif()

## EXPORTS
#ament_export_include_directories(
#  include
#)

#ament_export_libraries(
#  ${PROJECT_NAME}
#)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pal_statistics ros2_socketcan romea_implement_msgs)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  romea_core_common
  romea_core_mobile_base
  romea_common_utils
  romea_mobile_base_utils
  romea_mobile_base_hardware
)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
