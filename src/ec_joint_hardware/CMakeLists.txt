cmake_minimum_required(VERSION 3.14)
project(ec_joint_hardware VERSION 0.2.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_sdk 1.5.7 REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(ec_joint_hardware_plugins SHARED
  src/ec_joint_hardware.cpp
  src/joint_runtime.cpp
  src/utils.cpp
)
target_include_directories(ec_joint_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ec_joint_hardware_plugins PUBLIC cxx_std_17)
ament_target_dependencies(ec_joint_hardware_plugins PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)
target_link_libraries(ec_joint_hardware_plugins
  PUBLIC
    joint_sdk::joint_sdk
    nlohmann_json::nlohmann_json)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/joint_mode_slider.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_mode_slider)
install(
  TARGETS ec_joint_hardware_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface joint_sdk nlohmann_json pluginlib rclcpp rclcpp_lifecycle)
ament_package()
