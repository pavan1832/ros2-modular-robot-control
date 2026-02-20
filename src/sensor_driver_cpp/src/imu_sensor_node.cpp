/**
 * @file imu_sensor_node.cpp
 * @brief Implementation of the lifecycle IMU sensor driver node.
 */

#include "sensor_driver_cpp/imu_sensor_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace sensor_driver
{

// ── Constructor ──────────────────────────────────────────────────────────────

ImuSensorNode::ImuSensorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("imu_sensor_node", options),
  rng_(std::random_device{}())
{
  RCLCPP_INFO(get_logger(), "ImuSensorNode created (unconfigured)");
  declare_parameters();
}

// ── Parameter management ─────────────────────────────────────────────────────

void ImuSensorNode::declare_parameters()
{
  // Declare with descriptors so ros2 param describe works properly
  rcl_interfaces::msg::ParameterDescriptor rate_desc;
  rate_desc.description = "IMU publish rate in Hz";
  rate_desc.floating_point_range.resize(1);
  rate_desc.floating_point_range[0].from_value = 1.0;
  rate_desc.floating_point_range[0].to_value   = 1000.0;
  rate_desc.floating_point_range[0].step       = 1.0;
  declare_parameter<double>("publish_rate_hz", 100.0, rate_desc);

  rcl_interfaces::msg::ParameterDescriptor accel_noise_desc;
  accel_noise_desc.description = "Accelerometer white noise std-dev (m/s²)";
  declare_parameter<double>("accel_noise_stddev", 0.01, accel_noise_desc);

  rcl_interfaces::msg::ParameterDescriptor gyro_noise_desc;
  gyro_noise_desc.description = "Gyroscope white noise std-dev (rad/s)";
  declare_parameter<double>("gyro_noise_stddev", 0.001, gyro_noise_desc);

  declare_parameter<double>("accel_bias_drift_rate", 1e-5);
  declare_parameter<bool>("inject_fault", false);
  declare_parameter<std::string>("frame_id", "imu_link");
}

void ImuSensorNode::load_parameters()
{
  publish_rate_hz_       = get_parameter("publish_rate_hz").as_double();
  accel_noise_stddev_    = get_parameter("accel_noise_stddev").as_double();
  gyro_noise_stddev_     = get_parameter("gyro_noise_stddev").as_double();
  accel_bias_drift_rate_ = get_parameter("accel_bias_drift_rate").as_double();
  inject_fault_          = get_parameter("inject_fault").as_bool();
  frame_id_              = get_parameter("frame_id").as_string();

  RCLCPP_INFO(get_logger(),
    "Parameters loaded: rate=%.1f Hz, accel_noise=%.4f, gyro_noise=%.4f, fault=%s",
    publish_rate_hz_, accel_noise_stddev_, gyro_noise_stddev_,
    inject_fault_ ? "true" : "false");
}

// ── Lifecycle transitions ─────────────────────────────────────────────────────

ImuSensorNode::CallbackReturn
ImuSensorNode::on_configure(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_logger(), "Configuring IMU sensor node...");

  try {
    load_parameters();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_ERROR(get_logger(), "Parameter error during configure: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  // Create publishers (not yet activated / sending)
  imu_pub_ = create_lifecycle_publisher<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::QoS(10).reliable().transient_local());

  health_pub_ = create_lifecycle_publisher<std_msgs::msg::Bool>(
    "imu/is_healthy", rclcpp::QoS(1).reliable().transient_local());

  RCLCPP_INFO(get_logger(), "IMU sensor node configured successfully");
  return CallbackReturn::SUCCESS;
}

ImuSensorNode::CallbackReturn
ImuSensorNode::on_activate(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_logger(), "Activating IMU sensor node at %.1f Hz", publish_rate_hz_);

  imu_pub_->on_activate();
  health_pub_->on_activate();

  // Reset health state on activation
  is_healthy_.store(true);
  message_count_.store(0);
  consecutive_errors_ = 0;

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(period, [this]() { publish_imu(); });

  RCLCPP_INFO(get_logger(), "IMU sensor node active");
  return CallbackReturn::SUCCESS;
}

ImuSensorNode::CallbackReturn
ImuSensorNode::on_deactivate(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating IMU sensor node");

  publish_timer_->cancel();
  publish_timer_.reset();

  imu_pub_->on_deactivate();
  health_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "IMU sensor node deactivated (published %lu messages)",
    message_count_.load());
  return CallbackReturn::SUCCESS;
}

ImuSensorNode::CallbackReturn
ImuSensorNode::on_cleanup(const rclcpp_lifecycle::State & /*prev*/)
{
  imu_pub_.reset();
  health_pub_.reset();
  RCLCPP_INFO(get_logger(), "IMU sensor node cleaned up");
  return CallbackReturn::SUCCESS;
}

ImuSensorNode::CallbackReturn
ImuSensorNode::on_shutdown(const rclcpp_lifecycle::State & /*prev*/)
{
  if (publish_timer_) {
    publish_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "IMU sensor node shutdown");
  return CallbackReturn::SUCCESS;
}

ImuSensorNode::CallbackReturn
ImuSensorNode::on_error(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_ERROR(get_logger(), "IMU sensor node encountered unrecoverable error");
  return CallbackReturn::SUCCESS;
}

// ── Core publish logic ────────────────────────────────────────────────────────

void ImuSensorNode::publish_imu()
{
  // Fault injection: simulate sensor dropout
  if (inject_fault_) {
    ++consecutive_errors_;
    if (consecutive_errors_ >= kMaxConsecutiveErrors) {
      is_healthy_.store(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "FAULT INJECTED: IMU reporting unhealthy (consecutive errors: %lu)",
        consecutive_errors_);
    }

    auto health_msg = std_msgs::msg::Bool();
    health_msg.data = false;
    health_pub_->publish(health_msg);
    return;
  }

  // Normal operation: reset consecutive errors
  if (consecutive_errors_ > 0) {
    RCLCPP_INFO(get_logger(), "IMU recovered from fault injection");
    consecutive_errors_ = 0;
    is_healthy_.store(true);
  }

  update_bias_drift();

  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp    = now();
  msg.header.frame_id = frame_id_;

  // ── Simulated motion: gentle sinusoidal sway + gravity ───────────────────
  const double t = now().seconds();

  // Linear acceleration: gravity on Z + simulated sway
  msg.linear_acceleration.x = sample_gaussian(0.05 * std::sin(0.5 * t), accel_noise_stddev_)
                               + accel_bias_x_;
  msg.linear_acceleration.y = sample_gaussian(0.03 * std::cos(0.3 * t), accel_noise_stddev_)
                               + accel_bias_y_;
  msg.linear_acceleration.z = sample_gaussian(9.81, accel_noise_stddev_)
                               + accel_bias_z_;  // gravity

  // Angular velocity: slow rotation with noise
  msg.angular_velocity.x = sample_gaussian(0.0, gyro_noise_stddev_) + gyro_bias_x_;
  msg.angular_velocity.y = sample_gaussian(0.0, gyro_noise_stddev_) + gyro_bias_y_;
  msg.angular_velocity.z = sample_gaussian(0.01 * std::sin(0.1 * t), gyro_noise_stddev_)
                           + gyro_bias_z_;

  // Orientation: identity quaternion (a real driver would integrate or fuse)
  msg.orientation.w = 1.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation_covariance[0] = -1.0;  // -1 means orientation is not provided

  // Covariance diagonals (σ² for each axis)
  const double a_var = accel_noise_stddev_ * accel_noise_stddev_;
  const double g_var = gyro_noise_stddev_  * gyro_noise_stddev_;
  msg.linear_acceleration_covariance[0] = a_var;
  msg.linear_acceleration_covariance[4] = a_var;
  msg.linear_acceleration_covariance[8] = a_var;
  msg.angular_velocity_covariance[0]    = g_var;
  msg.angular_velocity_covariance[4]    = g_var;
  msg.angular_velocity_covariance[8]    = g_var;

  imu_pub_->publish(msg);

  // Publish health
  auto health_msg = std_msgs::msg::Bool();
  health_msg.data = true;
  health_pub_->publish(health_msg);

  is_healthy_.store(true);
  ++message_count_;

  RCLCPP_DEBUG(get_logger(),
    "IMU published #%lu: accel=[%.3f, %.3f, %.3f] gyro=[%.4f, %.4f, %.4f]",
    message_count_.load(),
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
    msg.angular_velocity.x,    msg.angular_velocity.y,    msg.angular_velocity.z);
}

void ImuSensorNode::update_bias_drift()
{
  // Simulate random walk bias drift (Allan deviation model simplified)
  accel_bias_x_ += sample_gaussian(0.0, accel_bias_drift_rate_);
  accel_bias_y_ += sample_gaussian(0.0, accel_bias_drift_rate_);
  accel_bias_z_ += sample_gaussian(0.0, accel_bias_drift_rate_);
  gyro_bias_x_  += sample_gaussian(0.0, accel_bias_drift_rate_ * 0.01);
  gyro_bias_y_  += sample_gaussian(0.0, accel_bias_drift_rate_ * 0.01);
  gyro_bias_z_  += sample_gaussian(0.0, accel_bias_drift_rate_ * 0.01);
}

double ImuSensorNode::sample_gaussian(double mean, double stddev)
{
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng_);
}

}  // namespace sensor_driver
