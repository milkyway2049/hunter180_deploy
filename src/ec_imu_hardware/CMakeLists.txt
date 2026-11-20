cmake_minimum_required(VERSION 3.20)
project(ec_imu_hardware VERSION 0.2.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(encos_driver 4.2.3 REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(ec_imu_hardware_plugins SHARED
  src/ec_imu_hardware.cpp
)
target_include_directories(ec_imu_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ec_imu_hardware_plugins PUBLIC cxx_std_17)
ament_target_dependencies(ec_imu_hardware_plugins PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)
target_link_libraries(ec_imu_hardware_plugins
  PUBLIC Encos::encos_driver
  PRIVATE Eigen3::Eigen)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ec_imu_hardware_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(encos_driver hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
