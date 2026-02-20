#pragma once

/**
 * @file actuator_controller_node.hpp
 * @brief Lifecycle-managed actuator controller ROS 2 node.
 *
 * Subscriptions:
 *   ~/actuator/command  (robot_interfaces/msg/ActuatorCommand)
 *
 * Publications:
 *   ~/actuator/state    (robot_interfaces/msg/ActuatorState)
 *
 * Services:
 *   ~/actuator/set_enabled  (robot_interfaces/srv/SetActuatorEnabled)
 *
 * Parameters:
 *   state_publish_rate_hz     double  50.0
 *   max_temperature_celsius   double  75.0
 *   max_joint_velocity_rad_s  double  3.14
 *   max_torque_nm             double  50.0
 *   dof_count                 int     6
 *   command_timeout_ms        int     500  (commands older than this are rejected)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "robot_interfaces/msg/actuator_command.hpp"
#include "robot_interfaces/msg/actuator_state.hpp"
#include "robot_interfaces/srv/set_actuator_enabled.hpp"
#include "actuator_controller_cpp/safety_monitor.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace actuator_controller
{

class ActuatorControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ActuatorControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ActuatorControllerNode() override = default;

  // ── Lifecycle ──────────────────────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)  override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)   override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)  override;

  // ── Test helpers ───────────────────────────────────────────────────────────
  bool is_enabled() const noexcept { return is_enabled_.load(); }
  bool is_safe()    const noexcept { return is_safe_.load(); }

private:
  void declare_parameters();
  void load_parameters();

  // ── Callbacks ──────────────────────────────────────────────────────────────
  void command_callback(const robot_interfaces::msg::ActuatorCommand::SharedPtr msg);
  void state_publish_callback();
  void set_enabled_callback(
    const std::shared_ptr<robot_interfaces::srv::SetActuatorEnabled::Request>  request,
          std::shared_ptr<robot_interfaces::srv::SetActuatorEnabled::Response> response);

  // ── Control logic ──────────────────────────────────────────────────────────
  void apply_command(const robot_interfaces::msg::ActuatorCommand & cmd);
  void simulate_motor_dynamics(double dt_sec);
  bool is_command_fresh(const robot_interfaces::msg::ActuatorCommand & cmd) const;

  // ── Publishers / Subscribers / Services ───────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::ActuatorState>::SharedPtr state_pub_;
  rclcpp::Subscription<robot_interfaces::msg::ActuatorCommand>::SharedPtr command_sub_;
  rclcpp::Service<robot_interfaces::srv::SetActuatorEnabled>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // ── State ──────────────────────────────────────────────────────────────────
  std::atomic<bool> is_enabled_{false};
  std::atomic<bool> is_safe_{true};

  std::mutex state_mutex_;  // protects joint_positions_ / velocities_ / torques_
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_torques_;
  double gripper_position_{0.0};
  double driver_temperature_{25.0};  // starts at ambient

  std::optional<robot_interfaces::msg::ActuatorCommand> last_command_;
  rclcpp::Time last_command_time_;

  // ── Parameters ─────────────────────────────────────────────────────────────
  double state_publish_rate_hz_{50.0};
  int    dof_count_{6};
  int    command_timeout_ms_{500};
  SafetyLimits safety_limits_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;
};

}  // namespace actuator_controller
