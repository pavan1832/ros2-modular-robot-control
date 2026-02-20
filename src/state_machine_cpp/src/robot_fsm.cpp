/**
 * @file robot_fsm.cpp
 * @brief Implementation of the pure C++ robot finite state machine.
 */

#include "state_machine_cpp/robot_fsm.hpp"

#include <stdexcept>

namespace state_machine
{

// ── String helpers ────────────────────────────────────────────────────────────

std::string to_string(RobotState state)
{
  switch (state) {
    case RobotState::IDLE:       return "IDLE";
    case RobotState::ACTIVE:     return "ACTIVE";
    case RobotState::ERROR:      return "ERROR";
    case RobotState::RECOVERING: return "RECOVERING";
    case RobotState::SHUTDOWN:   return "SHUTDOWN";
    default:                     return "UNKNOWN";
  }
}

std::string to_string(FsmEvent event)
{
  switch (event) {
    case FsmEvent::ACTIVATE:            return "ACTIVATE";
    case FsmEvent::DEACTIVATE:          return "DEACTIVATE";
    case FsmEvent::SENSOR_FAULT:        return "SENSOR_FAULT";
    case FsmEvent::ACTUATOR_FAULT:      return "ACTUATOR_FAULT";
    case FsmEvent::COMMUNICATION_FAULT: return "COMMUNICATION_FAULT";
    case FsmEvent::BEGIN_RECOVERY:      return "BEGIN_RECOVERY";
    case FsmEvent::RECOVERED:           return "RECOVERED";
    case FsmEvent::EMERGENCY_STOP:      return "EMERGENCY_STOP";
    case FsmEvent::SHUTDOWN_REQUEST:    return "SHUTDOWN_REQUEST";
    default:                            return "UNKNOWN_EVENT";
  }
}

// ── Constructor ──────────────────────────────────────────────────────────────

RobotFsm::RobotFsm(RobotState initial_state, StateChangeCallback on_transition)
: current_state_(initial_state),
  on_transition_cb_(std::move(on_transition)),
  state_entry_time_(std::chrono::steady_clock::now())
{}

// ── Guard helpers ─────────────────────────────────────────────────────────────

bool RobotFsm::can_activate(const FsmContext & ctx) const
{
  return ctx.sensor_healthy && ctx.actuator_healthy;
}

bool RobotFsm::can_recover(const FsmContext & ctx) const
{
  return ctx.sensor_healthy && ctx.actuator_healthy;
}

// ── Core FSM logic ────────────────────────────────────────────────────────────

TransitionResult RobotFsm::process_event(FsmEvent event, const FsmContext & ctx)
{
  // Emergency stop is valid from ANY state
  if (event == FsmEvent::EMERGENCY_STOP) {
    return do_transition(RobotState::ERROR,
      "Emergency stop triggered — all subsystems halted", true);
  }

  // Graceful shutdown valid from any non-error state
  if (event == FsmEvent::SHUTDOWN_REQUEST) {
    return do_transition(RobotState::SHUTDOWN, "Shutdown requested by operator");
  }

  switch (current_state_) {

    // ── IDLE ──────────────────────────────────────────────────────────────────
    case RobotState::IDLE:
      switch (event) {
        case FsmEvent::ACTIVATE:
          if (can_activate(ctx)) {
            return do_transition(RobotState::ACTIVE,
              "Activation request accepted: all subsystems healthy");
          }
          return {false, current_state_,
            "Cannot activate: subsystem not ready (sensor=" +
            std::to_string(ctx.sensor_healthy) +
            " actuator=" + std::to_string(ctx.actuator_healthy) + ")"};

        case FsmEvent::SENSOR_FAULT:
        case FsmEvent::ACTUATOR_FAULT:
        case FsmEvent::COMMUNICATION_FAULT:
          // In IDLE, faults are logged but don't change state (robot is stopped)
          return {false, current_state_,
            "Fault received in IDLE — logged but no transition needed"};

        default:
          return {false, current_state_,
            "Event " + to_string(event) + " not valid in IDLE"};
      }

    // ── ACTIVE ────────────────────────────────────────────────────────────────
    case RobotState::ACTIVE:
      switch (event) {
        case FsmEvent::DEACTIVATE:
          return do_transition(RobotState::IDLE,
            "Deactivation requested by operator/planner");

        case FsmEvent::SENSOR_FAULT:
          return do_transition(RobotState::ERROR,
            "Sensor fault detected while ACTIVE — safety halt", true);

        case FsmEvent::ACTUATOR_FAULT:
          return do_transition(RobotState::ERROR,
            "Actuator fault detected while ACTIVE — safety halt", true);

        case FsmEvent::COMMUNICATION_FAULT:
          return do_transition(RobotState::ERROR,
            "Communication timeout while ACTIVE — failsafe halt", true);

        default:
          return {false, current_state_,
            "Event " + to_string(event) + " not valid in ACTIVE"};
      }

    // ── ERROR ─────────────────────────────────────────────────────────────────
    case RobotState::ERROR:
      switch (event) {
        case FsmEvent::BEGIN_RECOVERY:
          return do_transition(RobotState::RECOVERING,
            "Recovery sequence initiated");

        default:
          return {false, current_state_,
            "In ERROR state — only BEGIN_RECOVERY or EMERGENCY_STOP are valid"};
      }

    // ── RECOVERING ────────────────────────────────────────────────────────────
    case RobotState::RECOVERING:
      switch (event) {
        case FsmEvent::RECOVERED:
          if (can_recover(ctx)) {
            return do_transition(RobotState::IDLE,
              "All subsystems healthy — returned to IDLE after recovery");
          }
          return {false, current_state_,
            "Recovery check failed: subsystems still unhealthy"};

        case FsmEvent::SENSOR_FAULT:
        case FsmEvent::ACTUATOR_FAULT:
          return do_transition(RobotState::ERROR,
            "New fault during recovery — aborting recovery", true);

        default:
          return {false, current_state_,
            "Event " + to_string(event) + " not valid in RECOVERING"};
      }

    // ── SHUTDOWN ──────────────────────────────────────────────────────────────
    case RobotState::SHUTDOWN:
      return {false, current_state_, "System is shutting down — no transitions accepted"};

    default:
      return {false, current_state_, "Unknown state in FSM — internal error"};
  }
}

TransitionResult RobotFsm::do_transition(
  RobotState to, const std::string & reason, bool count_as_fault)
{
  const RobotState from = current_state_;
  current_state_        = to;
  state_entry_time_     = std::chrono::steady_clock::now();

  if (count_as_fault) {
    ++total_fault_count_;
  }

  if (on_transition_cb_) {
    on_transition_cb_(from, to, reason);
  }

  return {true, to, reason};
}

double RobotFsm::time_in_current_state_sec() const
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - state_entry_time_).count();
}

void RobotFsm::reset()
{
  current_state_    = RobotState::IDLE;
  state_entry_time_ = std::chrono::steady_clock::now();
  total_fault_count_ = 0;
}

}  // namespace state_machine
