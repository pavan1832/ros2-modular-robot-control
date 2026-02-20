/**
 * @file main.cpp
 * @brief Entry point for the IMU sensor driver node.
 *
 * Uses rclcpp::executors::SingleThreadedExecutor so the lifecycle node
 * is responsive to lifecycle management service calls.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensor_driver_cpp/imu_sensor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);  // zero-copy within same process

  auto node = std::make_shared<sensor_driver::ImuSensorNode>(options);

  // Use a MultiThreadedExecutor to handle service calls and timer concurrently
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(node->get_logger(),
    "IMU sensor driver starting. Trigger lifecycle transitions via:"
    " ros2 lifecycle set /imu_sensor_node configure");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
