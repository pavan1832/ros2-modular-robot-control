/**
 * @file state_machine_node.cpp
 * @brief ROS 2 node wrapping the robot FSM.
 */

#include "state_machine_cpp/state_machine_node.hpp"

#include <functional>

namespace state_machine
{

StateMachineNode::StateMachineNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("state_machine_node", options)
{
  declare_parameters();
  RCLCPP_INFO(get_logger(), "StateMachineNode created");
}

// ── Parameters ────────────────────────────────────────────────────────────────

void StateMachineNode::declare_parameters()
{
  declare_parameter<double>("state_publish_rate_hz", 10.0);
  declare_parameter<double>("sensor_health_timeout_sec", 2.0);
  declare_parameter<double>("actuator_health_timeout_sec", 2.0);
  declare_parameter<bool>("auto_recovery_enabled", true);
  declare_parameter<double>("recovery_wait_sec", 5.0);
}

void StateMachineNode::load_parameters()
{
  state_publish_rate_hz_        = get_parameter("state_publish_rate_hz").as_double();
  sensor_health_timeout_sec_    = get_parameter("sensor_health_timeout_sec").as_double();
  actuator_health_timeout_sec_  = get_parameter("actuator_health_timeout_sec").as_double();
  auto_recovery_enabled_        = get_parameter("auto_recovery_enabled").as_bool();
  recovery_wait_sec_            = get_parameter("recovery_wait_sec").as_double();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

StateMachineNode::CallbackReturn
StateMachineNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring state machine...");
  load_parameters();

  // Create FSM with transition logging callback
  fsm_ = std::make_unique<RobotFsm>(
    RobotState::IDLE,
    [this](RobotState from, RobotState to, const std::string & reason) {
      on_fsm_transition(from, to, reason);
    });

  state_pub_ = create_lifecycle_publisher<robot_interfaces::msg::RobotState>(
    "robot_state", rclcpp::QoS(10).reliable().transient_local());

  sensor_health_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/imu_sensor_node/imu/is_healthy",
    rclcpp::QoS(5).reliable(),
    std::bind(&StateMachineNode::sensor_health_callback, this, std::placeholders::_1));

  actuator_state_sub_ = create_subscription<robot_interfaces::msg::ActuatorState>(
    "/actuator_controller_node/actuator/state",
    rclcpp::QoS(10).reliable(),
    std::bind(&StateMachineNode::actuator_state_callback, this, std::placeholders::_1));

  transition_srv_ = create_service<robot_interfaces::srv::TriggerStateTransition>(
    "trigger_transition",
    std::bind(&StateMachineNode::trigger_transition_callback,
      this, std::placeholders::_1, std::placeholders::_2));

  // Initialize timestamps to now so we get a grace period at startup
  last_sensor_update_   = now();
  last_actuator_update_ = now();

  RCLCPP_INFO(get_logger(), "State machine configured with %s recovery, recovery_wait=%.1f s",
    auto_recovery_enabled_ ? "AUTO" : "MANUAL", recovery_wait_sec_);

  return CallbackReturn::SUCCESS;
}

StateMachineNode::CallbackReturn
StateMachineNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating state machine");
  state_pub_->on_activate();

  const auto period = std::chrono::duration<double>(1.0 / state_publish_rate_hz_);
  evaluation_timer_ = create_wall_timer(period,
    [this]() { evaluation_callback(); });

  RCLCPP_INFO(get_logger(), "State machine ACTIVE — current FSM state: %s",
    fsm_->state_name().c_str());
  return CallbackReturn::SUCCESS;
}

StateMachineNode::CallbackReturn
StateMachineNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating state machine");
  evaluation_timer_->cancel();
  evaluation_timer_.reset();
  state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

StateMachineNode::CallbackReturn
StateMachineNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  fsm_.reset();
  state_pub_.reset();
  sensor_health_sub_.reset();
  actuator_state_sub_.reset();
  transition_srv_.reset();
  return CallbackReturn::SUCCESS;
}

StateMachineNode::CallbackReturn
StateMachineNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (evaluation_timer_) {
    evaluation_timer_->cancel();
  }
  if (fsm_) {
    fsm_->process_event(FsmEvent::SHUTDOWN_REQUEST);
  }
  RCLCPP_INFO(get_logger(), "State machine shutdown");
  return CallbackReturn::SUCCESS;
}

// ── Subscription callbacks ────────────────────────────────────────────────────

void StateMachineNode::sensor_health_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool was_healthy = sensor_healthy_.load();
  sensor_healthy_.store(msg->data);
  last_sensor_update_ = now();

  if (was_healthy && !msg->data) {
    RCLCPP_WARN(get_logger(), "Sensor health transitioned: HEALTHY → UNHEALTHY");
  } else if (!was_healthy && msg->data) {
    RCLCPP_INFO(get_logger(), "Sensor health transitioned: UNHEALTHY → HEALTHY");
  }
}

void StateMachineNode::actuator_state_callback(
  const robot_interfaces::msg::ActuatorState::SharedPtr msg)
{
  const bool was_healthy = actuator_healthy_.load();
  const bool is_healthy  = msg->is_safe && (msg->fault_code == 0);
  actuator_healthy_.store(is_healthy);
  last_actuator_update_ = now();

  if (was_healthy && !is_healthy) {
    RCLCPP_WARN(get_logger(),
      "Actuator health transitioned: HEALTHY → UNHEALTHY (fault_code=%u, temp=%.1f°C)",
      msg->fault_code, msg->driver_temperature_celsius);
  } else if (!was_healthy && is_healthy) {
    RCLCPP_INFO(get_logger(), "Actuator health transitioned: UNHEALTHY → HEALTHY");
  }
}

// ── Main evaluation loop ──────────────────────────────────────────────────────

void StateMachineNode::evaluation_callback()
{
  check_health_timeouts();

  const auto ctx    = build_context();
  const auto fstate = fsm_->current_state();

  // ── Drive automatic transitions based on health ───────────────────────────
  if (fstate == RobotState::ACTIVE) {
    if (!ctx.sensor_healthy) {
      fsm_->process_event(FsmEvent::SENSOR_FAULT, ctx);
    } else if (!ctx.actuator_healthy) {
      fsm_->process_event(FsmEvent::ACTUATOR_FAULT, ctx);
    }
  }

  // ── Auto recovery ─────────────────────────────────────────────────────────
  if (auto_recovery_enabled_) {
    if (fstate == RobotState::ERROR) {
      if (!recovery_start_time_.has_value()) {
        // Initiate recovery
        recovery_start_time_ = now();
        fsm_->process_event(FsmEvent::BEGIN_RECOVERY, ctx);
        RCLCPP_INFO(get_logger(), "Auto recovery started — waiting %.1f s", recovery_wait_sec_);
      }
    } else if (fstate == RobotState::RECOVERING) {
      if (recovery_start_time_.has_value()) {
        const double elapsed = (now() - *recovery_start_time_).seconds();
        if (elapsed >= recovery_wait_sec_ && ctx.sensor_healthy && ctx.actuator_healthy) {
          fsm_->process_event(FsmEvent::RECOVERED, ctx);
          recovery_start_time_.reset();
          RCLCPP_INFO(get_logger(), "Recovery complete — system returned to IDLE");
        } else if (!ctx.sensor_healthy || !ctx.actuator_healthy) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Recovery blocked: sensor=%s actuator=%s",
            ctx.sensor_healthy ? "OK" : "FAULT",
            ctx.actuator_healthy ? "OK" : "FAULT");
        }
      }
    } else {
      recovery_start_time_.reset();
    }
  }

  // ── Publish current state ─────────────────────────────────────────────────
  auto msg = robot_interfaces::msg::RobotState();
  msg.stamp              = now();
  msg.state              = static_cast<uint8_t>(fsm_->current_state());
  msg.state_name         = fsm_->state_name();
  msg.transition_reason  = last_transition_reason_;
  msg.time_in_state_sec  = fsm_->time_in_current_state_sec();
  msg.sensor_healthy     = ctx.sensor_healthy;
  msg.actuator_healthy   = ctx.actuator_healthy;
  msg.planner_healthy    = true;  // placeholder
  msg.total_fault_count  = fsm_->total_fault_count();
  state_pub_->publish(msg);

  RCLCPP_DEBUG(get_logger(),
    "FSM=%s sensor=%s actuator=%s faults=%u t=%.1fs",
    msg.state_name.c_str(),
    ctx.sensor_healthy ? "OK" : "FAULT",
    ctx.actuator_healthy ? "OK" : "FAULT",
    msg.total_fault_count,
    msg.time_in_state_sec);
}

// ── Service callback ──────────────────────────────────────────────────────────

void StateMachineNode::trigger_transition_callback(
  const std::shared_ptr<robot_interfaces::srv::TriggerStateTransition::Request> request,
        std::shared_ptr<robot_interfaces::srv::TriggerStateTransition::Response> response)
{
  const auto prev_state = fsm_->current_state();
  response->previous_state = static_cast<uint8_t>(prev_state);

  // Map requested target_state to an event
  FsmEvent event;
  switch (request->target_state) {
    case robot_interfaces::msg::RobotState::STATE_ACTIVE:
      event = FsmEvent::ACTIVATE;
      break;
    case robot_interfaces::msg::RobotState::STATE_IDLE:
      event = FsmEvent::DEACTIVATE;
      break;
    case robot_interfaces::msg::RobotState::STATE_ERROR:
      event = FsmEvent::EMERGENCY_STOP;
      break;
    case robot_interfaces::msg::RobotState::STATE_RECOVERING:
      event = FsmEvent::BEGIN_RECOVERY;
      break;
    default:
      response->success = false;
      response->message = "Unknown target state: " + std::to_string(request->target_state);
      return;
  }

  const auto ctx    = build_context();
  const auto result = fsm_->process_event(event, ctx);

  response->success   = result.success;
  response->message   = result.reason;
  response->new_state = static_cast<uint8_t>(fsm_->current_state());

  RCLCPP_INFO(get_logger(),
    "Manual transition request by '%s': %s → %s (%s)",
    request->reason.c_str(),
    to_string(prev_state).c_str(),
    to_string(fsm_->current_state()).c_str(),
    result.success ? "OK" : "REJECTED");
}

// ── Internal helpers ──────────────────────────────────────────────────────────

void StateMachineNode::check_health_timeouts()
{
  const double sensor_age   = (now() - last_sensor_update_).seconds();
  const double actuator_age = (now() - last_actuator_update_).seconds();

  if (sensor_age > sensor_health_timeout_sec_) {
    if (sensor_healthy_.load()) {
      RCLCPP_WARN(get_logger(),
        "Sensor health timeout (%.1f s since last update > %.1f s limit)",
        sensor_age, sensor_health_timeout_sec_);
    }
    sensor_healthy_.store(false);
  }

  if (actuator_age > actuator_health_timeout_sec_) {
    if (actuator_healthy_.load()) {
      RCLCPP_WARN(get_logger(),
        "Actuator health timeout (%.1f s since last update > %.1f s limit)",
        actuator_age, actuator_health_timeout_sec_);
    }
    actuator_healthy_.store(false);
  }
}

FsmContext StateMachineNode::build_context() const
{
  return FsmContext{
    .sensor_healthy   = sensor_healthy_.load(),
    .actuator_healthy = actuator_healthy_.load(),
    .planner_healthy  = true,
    .sensor_health_timeout_sec = sensor_health_timeout_sec_,
    .actuator_temperature_celsius = 25.0  // would come from actuator state
  };
}

void StateMachineNode::on_fsm_transition(
  RobotState from, RobotState to, const std::string & reason)
{
  last_transition_reason_ = reason;

  const char * level = (to == RobotState::ERROR) ? "ERROR" :
                       (to == RobotState::RECOVERING) ? "WARN" : "INFO";
  (void)level;

  if (to == RobotState::ERROR) {
    RCLCPP_ERROR(get_logger(), "[FSM] %s → %s | %s",
      to_string(from).c_str(), to_string(to).c_str(), reason.c_str());
  } else if (to == RobotState::RECOVERING) {
    RCLCPP_WARN(get_logger(), "[FSM] %s → %s | %s",
      to_string(from).c_str(), to_string(to).c_str(), reason.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "[FSM] %s → %s | %s",
      to_string(from).c_str(), to_string(to).c_str(), reason.c_str());
  }
}

RobotState StateMachineNode::current_fsm_state() const
{
  return fsm_ ? fsm_->current_state() : RobotState::IDLE;
}

}  // namespace state_machine
