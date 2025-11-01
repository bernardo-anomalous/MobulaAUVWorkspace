cmake_minimum_required(VERSION 3.8)
project(auv_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(auv_custom_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  auv_pkg/system_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
