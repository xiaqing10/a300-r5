cmake_minimum_required(VERSION 3.8)
project(a300_joystick)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(a300_joystick_mapper
  src/a300_joystick_mapper.cpp
)

ament_target_dependencies(a300_joystick_mapper
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  a300_joystick_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
