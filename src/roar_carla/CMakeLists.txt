cmake_minimum_required(VERSION 3.5)
project(roar_carla)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

include_directories(
  include
)
set(dependencies
  rclcpp
  rclcpp_action
  ackermann_msgs
  std_msgs
  carla_msgs
  nav_msgs
)

add_executable(carla_controller_converter src/carla_controller_converter.cpp)
ament_target_dependencies(carla_controller_converter
  ${dependencies}
)
target_include_directories(carla_controller_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS carla_controller_converter
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS carla_controller_converter
  DESTINATION lib/${PROJECT_NAME})

add_executable(carla_status_monitor_node src/carla_status_monitor_node.cpp)
ament_target_dependencies(carla_status_monitor_node
  ${dependencies}
)
target_include_directories(carla_status_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS carla_status_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS carla_status_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
