cmake_minimum_required(VERSION 3.5)
project(panda_switch_controller)

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(
SRC_FILES 
src/main_node.cpp
src/switch_controller.cpp
)

add_executable(switch_controller ${SRC_FILES})
ament_target_dependencies(switch_controller rclcpp std_msgs sensor_msgs)

install(TARGETS
  switch_controller
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
