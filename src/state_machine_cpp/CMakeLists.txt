cmake_minimum_required(VERSION 3.16)
project(state_machine_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)

# ── FSM library (testable independently of ROS) ───────────────────────────────
add_library(${PROJECT_NAME}_fsm SHARED
  src/robot_fsm.cpp
)

target_include_directories(${PROJECT_NAME}_fsm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_fsm robot_interfaces)

# ── ROS node library ──────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_lib SHARED
  src/state_machine_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_fsm)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  robot_interfaces
)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(state_machine_node src/main.cpp)
target_link_libraries(state_machine_node ${PROJECT_NAME}_lib)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_robot_fsm test/test_robot_fsm.cpp)
  target_link_libraries(test_robot_fsm ${PROJECT_NAME}_fsm)
  ament_target_dependencies(test_robot_fsm robot_interfaces)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}_fsm
  ${PROJECT_NAME}_lib
  state_machine_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
