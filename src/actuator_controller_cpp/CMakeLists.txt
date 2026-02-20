cmake_minimum_required(VERSION 3.16)
project(actuator_controller_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/actuator_controller_node.cpp
  src/safety_monitor.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  robot_interfaces
)

add_executable(actuator_controller_node src/main.cpp)
target_link_libraries(actuator_controller_node ${PROJECT_NAME}_lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_actuator_controller test/test_actuator_controller.cpp)
  target_link_libraries(test_actuator_controller ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_actuator_controller robot_interfaces)
endif()

install(TARGETS ${PROJECT_NAME}_lib actuator_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
