/**
 * @file main.cpp
 * @brief Entry point for the state machine node.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "state_machine_cpp/state_machine_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<state_machine::StateMachineNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(),
    "State machine node ready.\n"
    "  Trigger lifecycle: ros2 lifecycle set /state_machine_node configure\n"
    "  Manual transition: ros2 service call /state_machine_node/trigger_transition ...");

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
