cmake_minimum_required(VERSION 3.8)
project(my_can_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME}
  src/can_bus_interface.cpp
  src/can_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  can_msgs
  std_msgs
  hardware_interface
  pluginlib
)

add_executable(can_hardware_node src/main.cpp)
target_link_libraries(can_hardware_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} can_hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface can_hardware_interface.xml)

ament_package()
