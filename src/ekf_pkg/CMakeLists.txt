cmake_minimum_required(VERSION 3.10)
project(ekf_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ekf_input_adapter src/ekf_input_adapter.cpp)
ament_target_dependencies(
  ekf_input_adapter
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS ekf_input_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
