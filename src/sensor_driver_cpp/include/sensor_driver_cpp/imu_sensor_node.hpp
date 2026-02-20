#pragma once

/**
 * @file imu_sensor_node.hpp
 * @brief Lifecycle-managed mock IMU sensor driver for ROS 2.
 *
 * Publishes sensor_msgs/Imu messages with configurable Gaussian noise,
 * bias drift, and fault injection. Designed to mirror a real IMU driver
 * so integration testing and simulation use the same node interface.
 *
 * Topic:   ~/imu/data        (sensor_msgs/msg/Imu)
 * Topic:   ~/imu/is_healthy  (std_msgs/msg/Bool)
 * Param:   publish_rate_hz   (double, default 100.0)
 * Param:   accel_noise_stddev (double, default 0.01 m/s²)
 * Param:   gyro_noise_stddev  (double, default 0.001 rad/s)
 * Param:   inject_fault       (bool,   default false)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <random>
#include <chrono>
#include <memory>
#include <string>
#include <atomic>

namespace sensor_driver
{

/**
 * @class ImuSensorNode
 * @brief Lifecycle node that publishes mock IMU data.
 *
 * Lifecycle states:
 *   unconfigured → configured → active → deactivated → finalized
 *
 * In ACTIVE state the timer fires at publish_rate_hz and emits IMU messages.
 * In any other state the timer is cancelled and no messages are sent.
 */
class ImuSensorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @param options  NodeOptions forwarded from main (enables composition)
  explicit ImuSensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ImuSensorNode() override = default;

  // ── Lifecycle callbacks ────────────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  // ── Public accessors (used by unit tests) ──────────────────────────────────
  bool is_healthy() const noexcept { return is_healthy_.load(); }
  uint64_t message_count() const noexcept { return message_count_.load(); }

private:
  // ── Internal helpers ───────────────────────────────────────────────────────
  void declare_parameters();
  void load_parameters();
  void publish_imu();
  double sample_gaussian(double mean, double stddev);

  // Simulate slow sensor bias drift (common in real MEMS IMUs)
  void update_bias_drift();

  // ── Publishers ─────────────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr health_pub_;

  // ── Timer ──────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ── Parameters (loaded in on_configure) ───────────────────────────────────
  double publish_rate_hz_{100.0};
  double accel_noise_stddev_{0.01};   // m/s²
  double gyro_noise_stddev_{0.001};   // rad/s
  double accel_bias_drift_rate_{1e-5}; // m/s² per tick
  bool   inject_fault_{false};
  std::string frame_id_{"imu_link"};

  // ── Noise / drift state ────────────────────────────────────────────────────
  std::mt19937_64 rng_;
  double accel_bias_x_{0.0}, accel_bias_y_{0.0}, accel_bias_z_{0.0};
  double gyro_bias_x_{0.0},  gyro_bias_y_{0.0},  gyro_bias_z_{0.0};

  // ── Health tracking ────────────────────────────────────────────────────────
  std::atomic<bool>     is_healthy_{true};
  std::atomic<uint64_t> message_count_{0};
  uint64_t consecutive_errors_{0};
  static constexpr uint64_t kMaxConsecutiveErrors = 5;
};

}  // namespace sensor_driver
