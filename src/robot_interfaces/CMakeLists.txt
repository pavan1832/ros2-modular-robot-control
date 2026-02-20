cmake_minimum_required(VERSION 3.16)
project(robot_interfaces VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/ActuatorState.msg"
  "msg/RobotState.msg"
  "srv/SetActuatorEnabled.srv"
  "srv/TriggerStateTransition.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
