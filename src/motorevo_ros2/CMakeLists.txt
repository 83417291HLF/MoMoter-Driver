cmake_minimum_required(VERSION 3.8)
project(motorevo_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(motorevo_driver SHARED
  src/protocol.cpp
  src/motor_bus.cpp
  src/meow_usb_transport.cpp
  src/socketcan_transport.cpp
)
target_include_directories(motorevo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motorevo_driver ${CMAKE_DL_LIBS})

add_executable(motorevo_node src/motorevo_node.cpp)
target_link_libraries(motorevo_node motorevo_driver)
ament_target_dependencies(motorevo_node
  diagnostic_msgs rclcpp sensor_msgs std_srvs)

add_library(motorevo_system SHARED src/motorevo_system.cpp)
target_link_libraries(motorevo_system motorevo_driver)
ament_target_dependencies(motorevo_system
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface motorevo_ros2_control.xml)

install(TARGETS motorevo_driver motorevo_system
  EXPORT export_motorevo_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS motorevo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf scripts vendor DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
install(PROGRAMS scripts/setup_meow_usb2fdcan.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES motorevo_ros2_control.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol motorevo_driver)
endif()

ament_export_targets(export_motorevo_driver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(motorevo_driver)
ament_package()
