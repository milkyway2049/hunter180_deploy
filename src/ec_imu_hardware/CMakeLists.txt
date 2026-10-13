cmake_minimum_required(VERSION 3.20)
project(ec_imu_hardware VERSION 0.2.0 LANGUAGES CXX)

option(BUILD_TESTING "Build tests for ec_imu_hardware" OFF)

find_package(ament_cmake REQUIRED)
find_package(encos_driver REQUIRED CONFIG)
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(IMU_HARDWARE_TEST_PLUGIN_DIR "${CMAKE_CURRENT_BINARY_DIR}/test_plugins")
  add_library(ec_imu_hardware_test_can_plugin SHARED
    test/ec_imu_hardware_test_can_plugin.cpp)
  target_compile_features(ec_imu_hardware_test_can_plugin PUBLIC cxx_std_17)
  target_link_libraries(ec_imu_hardware_test_can_plugin PRIVATE Encos::encos_driver)
  set_target_properties(ec_imu_hardware_test_can_plugin PROPERTIES
    OUTPUT_NAME CanPlugin
    LIBRARY_OUTPUT_DIRECTORY "${IMU_HARDWARE_TEST_PLUGIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${IMU_HARDWARE_TEST_PLUGIN_DIR}")

  ament_add_gtest(ec_imu_hardware_test
    test/ec_imu_hardware_test.cpp)
  target_compile_definitions(ec_imu_hardware_test PRIVATE
    IMU_HARDWARE_TEST_PLUGIN_PATH="${IMU_HARDWARE_TEST_PLUGIN_DIR}")
  target_link_libraries(ec_imu_hardware_test
    ec_imu_hardware_plugins
    ec_imu_hardware_test_can_plugin)
  add_dependencies(ec_imu_hardware_test ec_imu_hardware_test_can_plugin)
endif()

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
