cmake_minimum_required(VERSION 3.16)
project(robot_bringup VERSION 1.0.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
