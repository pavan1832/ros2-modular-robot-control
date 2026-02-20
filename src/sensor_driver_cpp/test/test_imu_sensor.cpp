/**
 * @file test_imu_sensor.cpp
 * @brief Unit tests for ImuSensorNode using GoogleTest.
 *
 * Tests focus on the node's internal logic that is independently testable
 * without a live ROS 2 graph.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_driver_cpp/imu_sensor_node.hpp"

class ImuSensorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("publish_rate_hz", 10.0);
    opts.append_parameter_override("accel_noise_stddev", 0.001);
    opts.append_parameter_override("gyro_noise_stddev", 0.0001);
    opts.append_parameter_override("inject_fault", false);
    node_ = std::make_shared<sensor_driver::ImuSensorNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<sensor_driver::ImuSensorNode> node_;
};

TEST_F(ImuSensorNodeTest, NodeCreatesSuccessfully)
{
  ASSERT_NE(node_, nullptr);
  EXPECT_EQ(node_->get_name(), std::string("imu_sensor_node"));
}

TEST_F(ImuSensorNodeTest, InitialHealthStateIsTrue)
{
  EXPECT_TRUE(node_->is_healthy());
}

TEST_F(ImuSensorNodeTest, InitialMessageCountIsZero)
{
  EXPECT_EQ(node_->message_count(), 0u);
}

TEST_F(ImuSensorNodeTest, LifecycleConfigureSucceeds)
{
  auto result = node_->trigger_transition(
    rclcpp_lifecycle::Transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  EXPECT_EQ(result.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ImuSensorNodeTest, LifecycleActivateSucceeds)
{
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  auto result = node_->trigger_transition(
    rclcpp_lifecycle::Transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

  EXPECT_EQ(result.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(ImuSensorNodeTest, NodePublishesAfterActivation)
{
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

  // Spin for 500ms at 10 Hz -> expect ~5 messages
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_->get_node_base_interface());

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
    exec.spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_GE(node_->message_count(), 3u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
