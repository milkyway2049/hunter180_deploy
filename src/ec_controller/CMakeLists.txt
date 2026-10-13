cmake_minimum_required(VERSION 3.8)
project(ec_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CONTROLLER_INCLUDE_DEPENDS
  ament_index_cpp
  pluginlib
  controller_interface
  rclcpp
  std_msgs
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ec_radio REQUIRED)

foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ONNX Runtime (vendored thirdparty)
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_DIR}/lib")

# Set RPATH so the runtime linker can find ONNX Runtime
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

add_library(
  ec_controller
  SHARED
  src/ControllerBase.cpp
  src/DefaultController.cpp
  src/StandController.cpp
  src/WalkController.cpp
)

target_compile_features(ec_controller PUBLIC cxx_std_17)
target_include_directories(ec_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ec_controller>
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_link_directories(ec_controller PRIVATE ${ONNXRUNTIME_LIB_DIR})
target_link_libraries(ec_controller PRIVATE onnxruntime Threads::Threads)

ament_target_dependencies(
  ec_controller
  PUBLIC
  ${CONTROLLER_INCLUDE_DEPENDS}
  ec_radio
)

target_compile_definitions(ec_controller PRIVATE "ec_controller_BUILDING_DLL")

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ec_controller.xml)

# INSTALL
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ec_controller.xml
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ec_controller
  EXPORT export_ec_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install ONNX Runtime shared libraries alongside the controller library
install(FILES
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1.28.1"
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime_providers_shared.so"
  DESTINATION lib
)
install(CODE "
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    libonnxruntime.so.1.28.1
    \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/libonnxruntime.so.1\")
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    libonnxruntime.so.1
    \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/libonnxruntime.so\")
")

add_executable(controller_switcher_node
  src/controller_switcher_node.cpp
)
target_compile_features(controller_switcher_node PUBLIC cxx_std_17)
ament_target_dependencies(controller_switcher_node
  rclcpp
  std_msgs
  geometry_msgs
  controller_manager_msgs
  ec_radio
)
install(TARGETS controller_switcher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_ec_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS} ec_radio)
ament_package()
