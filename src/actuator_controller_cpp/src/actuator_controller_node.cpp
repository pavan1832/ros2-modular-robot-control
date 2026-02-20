/**
 * @file actuator_controller_node.cpp
 * @brief Actuator controller node implementation.
 */

#include "actuator_controller_cpp/actuator_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace actuator_controller
{

// ── Constructor ──────────────────────────────────────────────────────────────

ActuatorControllerNode::ActuatorControllerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("actuator_controller_node", options)
{
  declare_parameters();
  RCLCPP_INFO(get_logger(), "ActuatorControllerNode created");
}

// ── Parameter management ─────────────────────────────────────────────────────

void ActuatorControllerNode::declare_parameters()
{
  declare_parameter<double>("state_publish_rate_hz", 50.0);
  declare_parameter<int>("dof_count", 6);
  declare_parameter<int>("command_timeout_ms", 500);
  declare_parameter<double>("max_temperature_celsius", 75.0);
  declare_parameter<double>("emergency_stop_temperature", 85.0);
  declare_parameter<double>("max_joint_velocity_rad_s", 3.14159);
  declare_parameter<double>("max_torque_nm", 50.0);
  declare_parameter<double>("max_speed_fraction", 1.0);
}

void ActuatorControllerNode::load_parameters()
{
  state_publish_rate_hz_ = get_parameter("state_publish_rate_hz").as_double();
  dof_count_             = get_parameter("dof_count").as_int();
  command_timeout_ms_    = get_parameter("command_timeout_ms").as_int();

  safety_limits_.max_temperature_celsius   = get_parameter("max_temperature_celsius").as_double();
  safety_limits_.emergency_stop_temperature = get_parameter("emergency_stop_temperature").as_double();
  safety_limits_.max_joint_velocity_rad_s  = get_parameter("max_joint_velocity_rad_s").as_double();
  safety_limits_.max_torque_nm             = get_parameter("max_torque_nm").as_double();
  safety_limits_.max_speed_fraction        = get_parameter("max_speed_fraction").as_double();

  // Initialize joint state vectors
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_positions_.assign(static_cast<size_t>(dof_count_), 0.0);
    joint_velocities_.assign(static_cast<size_t>(dof_count_), 0.0);
    joint_torques_.assign(static_cast<size_t>(dof_count_), 0.0);
  }

  safety_monitor_ = std::make_unique<SafetyMonitor>(safety_limits_);

  RCLCPP_INFO(get_logger(),
    "Parameters: rate=%.1f Hz, DOF=%d, timeout=%d ms, max_temp=%.1f°C",
    state_publish_rate_hz_, dof_count_, command_timeout_ms_,
    safety_limits_.max_temperature_celsius);
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

ActuatorControllerNode::CallbackReturn
ActuatorControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring actuator controller...");
  load_parameters();

  // State publisher
  state_pub_ = create_lifecycle_publisher<robot_interfaces::msg::ActuatorState>(
    "actuator/state", rclcpp::QoS(10).reliable());

  // Command subscriber — use a dedicated callback group for real-time priority
  auto cmd_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cmd_cb_group;

  command_sub_ = create_subscription<robot_interfaces::msg::ActuatorCommand>(
    "actuator/command",
    rclcpp::QoS(10).reliable(),
    std::bind(&ActuatorControllerNode::command_callback, this, std::placeholders::_1),
    sub_opts);

  // Enable/disable service
  enable_service_ = create_service<robot_interfaces::srv::SetActuatorEnabled>(
    "actuator/set_enabled",
    std::bind(&ActuatorControllerNode::set_enabled_callback,
      this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Actuator controller configured");
  return CallbackReturn::SUCCESS;
}

ActuatorControllerNode::CallbackReturn
ActuatorControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating actuator controller");
  state_pub_->on_activate();
  last_command_time_ = now();

  const auto period = std::chrono::duration<double>(1.0 / state_publish_rate_hz_);
  state_timer_ = create_wall_timer(period,
    [this]() { state_publish_callback(); });

  RCLCPP_INFO(get_logger(), "Actuator controller active (disabled by default — use service to enable)");
  return CallbackReturn::SUCCESS;
}

ActuatorControllerNode::CallbackReturn
ActuatorControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating actuator controller");
  is_enabled_.store(false);
  state_timer_->cancel();
  state_timer_.reset();
  state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

ActuatorControllerNode::CallbackReturn
ActuatorControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  state_pub_.reset();
  command_sub_.reset();
  enable_service_.reset();
  safety_monitor_.reset();
  RCLCPP_INFO(get_logger(), "Actuator controller cleaned up");
  return CallbackReturn::SUCCESS;
}

ActuatorControllerNode::CallbackReturn
ActuatorControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  is_enabled_.store(false);
  if (state_timer_) {
    state_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "Actuator controller shutdown");
  return CallbackReturn::SUCCESS;
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

void ActuatorControllerNode::command_callback(
  const robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
{
  if (!is_enabled_.load()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Received command but actuator is DISABLED — ignoring");
    return;
  }

  if (!is_command_fresh(*msg)) {
    RCLCPP_WARN(get_logger(),
      "Stale command from '%s' rejected (too old)", msg->source_node.c_str());
    return;
  }

  // ── Safety pre-check ────────────────────────────────────────────────────
  auto safety_result = safety_monitor_->check_all(
    driver_temperature_,
    msg->velocity_targets,
    msg->torque_limits,
    msg->gripper_position,
    msg->speed_fraction);

  if (!safety_result.safe) {
    RCLCPP_ERROR(get_logger(), "Safety violation [code %u]: %s",
      safety_result.fault_code, safety_result.message.c_str());
    is_safe_.store(false);
    is_enabled_.store(false);
    return;
  }

  apply_command(*msg);
  last_command_time_ = now();

  RCLCPP_DEBUG(get_logger(),
    "Command applied: gripper=%.2f, speed_frac=%.2f, source=%s",
    msg->gripper_position, msg->speed_fraction, msg->source_node.c_str());
}

void ActuatorControllerNode::state_publish_callback()
{
  // Simulate motor dynamics (integrate positions)
  const double dt = 1.0 / state_publish_rate_hz_;
  simulate_motor_dynamics(dt);

  // Run periodic safety check (thermal check on every publish tick)
  auto temp_check = safety_monitor_->check_temperature(driver_temperature_);
  if (!temp_check.safe) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Thermal safety violation: %s", temp_check.message.c_str());
    is_safe_.store(false);
    is_enabled_.store(false);
  }

  auto msg = robot_interfaces::msg::ActuatorState();
  msg.stamp = now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    msg.current_positions  = joint_positions_;
    msg.current_velocities = joint_velocities_;
    msg.current_torques    = joint_torques_;
  }

  msg.gripper_position          = gripper_position_;
  msg.is_enabled                = is_enabled_.load();
  msg.is_safe                   = is_safe_.load();
  msg.driver_temperature_celsius = driver_temperature_;
  msg.fault_code = is_safe_.load() ?
    robot_interfaces::msg::ActuatorState::FAULT_NONE :
    robot_interfaces::msg::ActuatorState::FAULT_OVER_TEMPERATURE;

  state_pub_->publish(msg);
}

void ActuatorControllerNode::set_enabled_callback(
  const std::shared_ptr<robot_interfaces::srv::SetActuatorEnabled::Request>  request,
        std::shared_ptr<robot_interfaces::srv::SetActuatorEnabled::Response> response)
{
  if (request->enable) {
    // Check safety before enabling
    auto safety_result = safety_monitor_->check_temperature(driver_temperature_);
    if (!safety_result.safe) {
      response->success    = false;
      response->message    = "Cannot enable: " + safety_result.message;
      response->fault_code = safety_result.fault_code;
      RCLCPP_WARN(get_logger(), "Enable request denied: %s", response->message.c_str());
      return;
    }
    is_enabled_.store(true);
    is_safe_.store(true);
    response->success = true;
    response->message = "Actuator ENABLED. Reason: " + request->reason;
    RCLCPP_INFO(get_logger(), "Actuator enabled by: %s", request->reason.c_str());
  } else {
    is_enabled_.store(false);
    response->success = true;
    response->message = "Actuator DISABLED. Reason: " + request->reason;
    RCLCPP_INFO(get_logger(), "Actuator disabled by: %s", request->reason.c_str());
  }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

void ActuatorControllerNode::apply_command(const robot_interfaces::msg::ActuatorCommand & cmd)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Resize targets if needed (first command after DOF change)
  const size_t dof = static_cast<size_t>(dof_count_);

  if (!cmd.position_targets.empty()) {
    for (size_t i = 0; i < std::min(dof, cmd.position_targets.size()); ++i) {
      // Simple first-order filter: 20% of commanded position per cycle
      joint_positions_[i] += 0.2 * (cmd.position_targets[i] - joint_positions_[i]);
    }
  }

  if (!cmd.velocity_targets.empty()) {
    for (size_t i = 0; i < std::min(dof, cmd.velocity_targets.size()); ++i) {
      joint_velocities_[i] = cmd.velocity_targets[i] * cmd.speed_fraction;
    }
  }

  if (!cmd.torque_limits.empty()) {
    for (size_t i = 0; i < std::min(dof, cmd.torque_limits.size()); ++i) {
      joint_torques_[i] = cmd.torque_limits[i];
    }
  }

  gripper_position_ = cmd.gripper_position;
}

void ActuatorControllerNode::simulate_motor_dynamics(double dt_sec)
{
  // Slowly ramp temperature when enabled (motor heating), cool when idle
  std::lock_guard<std::mutex> lock(state_mutex_);
  const double ambient_temp   = 25.0;
  const double heat_rate      = 0.05;  // °C per tick when active
  const double cool_rate      = 0.02;  // °C per tick when idle

  if (is_enabled_.load()) {
    driver_temperature_ += heat_rate * dt_sec * state_publish_rate_hz_;
  } else {
    driver_temperature_ = std::max(ambient_temp,
      driver_temperature_ - cool_rate * dt_sec * state_publish_rate_hz_);
  }

  // Integrate velocities → positions (for mock simulation)
  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    joint_positions_[i] += joint_velocities_[i] * dt_sec;
    // Clamp to ±2π
    joint_positions_[i] = std::clamp(joint_positions_[i], -6.28, 6.28);
  }
}

bool ActuatorControllerNode::is_command_fresh(
  const robot_interfaces::msg::ActuatorCommand & cmd) const
{
  const auto cmd_time = rclcpp::Time(cmd.stamp);
  const auto age_ms   = (now() - cmd_time).nanoseconds() / 1'000'000;
  return age_ms <= static_cast<int64_t>(command_timeout_ms_);
}

}  // namespace actuator_controller
