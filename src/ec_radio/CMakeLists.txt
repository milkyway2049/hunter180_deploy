cmake_minimum_required(VERSION 3.8)
project(ec_radio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Radio.msg")

add_executable(radio_node
  src/radio_node.cpp
)
target_compile_features(radio_node PUBLIC cxx_std_17)

ament_target_dependencies(radio_node
  rclcpp
  serial_driver
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(radio_node
  ${cpp_typesupport_target}
)

install(TARGETS radio_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
