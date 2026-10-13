cmake_minimum_required(VERSION 3.14)
project(ec_joint_hardware VERSION 0.2.0 LANGUAGES CXX)

option(BUILD_TESTING "Build tests for ec_joint_hardware" OFF)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_sdk REQUIRED CONFIG)
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(ec_joint_hardware_config_test
    test/ec_joint_hardware_config_test.cpp
    src/utils.cpp)
  target_include_directories(ec_joint_hardware_config_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(ec_joint_hardware_config_test
    joint_sdk::joint_sdk
    nlohmann_json::nlohmann_json)

  ament_add_gtest(ec_joint_hardware_joint_runtime_test
    test/ec_joint_hardware_joint_runtime_test.cpp
    src/joint_runtime.cpp
    src/utils.cpp)
  target_include_directories(ec_joint_hardware_joint_runtime_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(ec_joint_hardware_joint_runtime_test
    joint_sdk::joint_sdk
    nlohmann_json::nlohmann_json)

  ament_add_gtest(ec_joint_hardware_hardware_test
    test/ec_joint_hardware_hardware_test.cpp
    src/ec_joint_hardware.cpp
    src/joint_runtime.cpp
    src/utils.cpp)
  target_include_directories(ec_joint_hardware_hardware_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(ec_joint_hardware_hardware_test
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle)
  target_link_libraries(ec_joint_hardware_hardware_test joint_sdk::joint_sdk nlohmann_json::nlohmann_json)

  ament_add_pytest_test(ec_joint_hardware_joint_mode_slider_test
    test/test_joint_mode_slider.py
    TIMEOUT 30)
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
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
