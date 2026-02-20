/**
 * @file main.cpp
 * @brief Entry point for the actuator controller node.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "actuator_controller_cpp/actuator_controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<actuator_controller::ActuatorControllerNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, 2);  // 2 threads: timer + service callbacks
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(),
    "Actuator controller ready. Configure/activate via lifecycle, then:\n"
    "  ros2 service call /actuator/set_enabled robot_interfaces/srv/SetActuatorEnabled"
    " '{enable: true, reason: operator}'");

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
