cmake_minimum_required(VERSION 3.5)
project(final_project)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY final_project
  DESTINATION lib/${final_project}
)

ament_package()
