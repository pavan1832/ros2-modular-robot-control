#pragma once

/**
 * @file state_machine_node.hpp
 * @brief ROS 2 lifecycle node wrapping the RobotFsm.
 *
 * Subscriptions:
 *   /imu_sensor_node/imu/is_healthy    (std_msgs/msg/Bool)
 *   /actuator_controller_node/actuator/state  (robot_interfaces/msg/ActuatorState)
 *
 * Publications:
 *   ~/robot_state                      (robot_interfaces/msg/RobotState)
 *
 * Services:
 *   ~/trigger_transition               (robot_interfaces/srv/TriggerStateTransition)
 *
 * Parameters:
 *   state_publish_rate_hz  double  10.0
 *   sensor_health_timeout_sec  double  2.0
 *   actuator_health_timeout_sec double  2.0
 *   auto_recovery_enabled  bool    true
 *   recovery_wait_sec      double  5.0
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>

#include "robot_interfaces/msg/actuator_state.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "robot_interfaces/srv/trigger_state_transition.hpp"
#include "state_machine_cpp/robot_fsm.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>

namespace state_machine
{

class StateMachineNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit StateMachineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StateMachineNode() override = default;

  // Lifecycle
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)  override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)   override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)  override;

  // Test accessors
  RobotState current_fsm_state() const;

private:
  void declare_parameters();
  void load_parameters();

  // Subscription callbacks
  void sensor_health_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void actuator_state_callback(const robot_interfaces::msg::ActuatorState::SharedPtr msg);

  // Timer callback: evaluate health and drive FSM
  void evaluation_callback();

  // Service callback: manual transition override
  void trigger_transition_callback(
    const std::shared_ptr<robot_interfaces::srv::TriggerStateTransition::Request>,
          std::shared_ptr<robot_interfaces::srv::TriggerStateTransition::Response>);

  // Internal helpers
  void check_health_timeouts();
  FsmContext build_context() const;
  void publish_robot_state(const robot_interfaces::msg::RobotState & state_msg);
  void on_fsm_transition(RobotState from, RobotState to, const std::string & reason);

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::RobotState>::SharedPtr state_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_health_sub_;
  rclcpp::Subscription<robot_interfaces::msg::ActuatorState>::SharedPtr actuator_state_sub_;

  // Services
  rclcpp::Service<robot_interfaces::srv::TriggerStateTransition>::SharedPtr transition_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr evaluation_timer_;

  // FSM
  std::unique_ptr<RobotFsm> fsm_;

  // Health tracking
  std::atomic<bool> sensor_healthy_{false};
  std::atomic<bool> actuator_healthy_{false};
  rclcpp::Time last_sensor_update_;
  rclcpp::Time last_actuator_update_;
  std::string last_transition_reason_{"initialized"};

  // Parameters
  double state_publish_rate_hz_{10.0};
  double sensor_health_timeout_sec_{2.0};
  double actuator_health_timeout_sec_{2.0};
  bool   auto_recovery_enabled_{true};
  double recovery_wait_sec_{5.0};

  // Recovery timer tracking
  std::optional<rclcpp::Time> recovery_start_time_;
};

}  // namespace state_machine
