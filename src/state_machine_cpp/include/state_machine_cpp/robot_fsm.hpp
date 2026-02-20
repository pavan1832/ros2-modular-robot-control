#pragma once

/**
 * @file robot_fsm.hpp
 * @brief Pure C++ finite state machine for robot behavior control.
 *
 * Deliberately kept free of ROS dependencies so it can be unit-tested
 * without a ROS environment. The ROS node wraps this class and feeds
 * it external events from subscribed topics.
 *
 * State diagram:
 *
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │                                                             │
 *   ▼                                                             │
 * [IDLE] ──activate()──► [ACTIVE] ──fault()──► [ERROR]          │
 *   ▲                       │  ▲                  │              │
 *   │                       │  │                  │              │
 *   └──deactivate()──────────┘  └──recovered()────┘              │
 *                                    [RECOVERING]◄───────────────┘
 *
 * Fault recovery flow:
 *   ERROR → begin_recovery() → RECOVERING → recovered() → IDLE
 *
 * Emergency stop:
 *   ANY STATE → emergency_stop() → ERROR
 */

#include <chrono>
#include <functional>
#include <string>
#include <optional>
#include <cstdint>

namespace state_machine
{

/// Robot FSM states — values match robot_interfaces/msg/RobotState constants
enum class RobotState : uint8_t
{
  IDLE       = 0,
  ACTIVE     = 1,
  ERROR      = 2,
  RECOVERING = 3,
  SHUTDOWN   = 4,
};

std::string to_string(RobotState state);

/// Events that can trigger transitions
enum class FsmEvent
{
  ACTIVATE,           // Operator/planner requests activation
  DEACTIVATE,         // Operator/planner requests return to idle
  SENSOR_FAULT,       // Sensor health check failed
  ACTUATOR_FAULT,     // Actuator reported fault
  COMMUNICATION_FAULT,// Heartbeat/comm timeout
  BEGIN_RECOVERY,     // Automatic or operator-initiated recovery
  RECOVERED,          // All subsystems healthy again
  EMERGENCY_STOP,     // Hard E-stop — always goes to ERROR
  SHUTDOWN_REQUEST,   // Graceful shutdown
};

std::string to_string(FsmEvent event);

/// Context passed into each state transition (read-only snapshot)
struct FsmContext
{
  bool   sensor_healthy{true};
  bool   actuator_healthy{true};
  bool   planner_healthy{true};
  double sensor_health_timeout_sec{5.0};
  double actuator_temperature_celsius{25.0};
};

/// Result of a transition attempt
struct TransitionResult
{
  bool        success{false};
  RobotState  new_state;
  std::string reason;
};

/// Type alias for state-change callbacks
using StateChangeCallback = std::function<void(RobotState from, RobotState to, const std::string & reason)>;

/**
 * @class RobotFsm
 * @brief Thread-safe finite state machine for robot behavior.
 *
 * All public methods are thread-safe. Callers must hold no external locks.
 */
class RobotFsm
{
public:
  explicit RobotFsm(
    RobotState initial_state = RobotState::IDLE,
    StateChangeCallback on_transition = nullptr);

  // ── Event-driven transitions ───────────────────────────────────────────────
  TransitionResult process_event(FsmEvent event, const FsmContext & ctx = {});

  // ── Guard helpers ──────────────────────────────────────────────────────────
  bool can_activate(const FsmContext & ctx) const;
  bool can_recover(const FsmContext & ctx) const;

  // ── State access ───────────────────────────────────────────────────────────
  RobotState current_state()  const noexcept { return current_state_; }
  std::string state_name()    const { return to_string(current_state_); }
  uint32_t total_fault_count()const noexcept { return total_fault_count_; }

  /// Seconds spent in current state
  double time_in_current_state_sec() const;

  /// Reset to IDLE (for testing / operator override)
  void reset();

private:
  TransitionResult do_transition(
    RobotState to, const std::string & reason, bool count_as_fault = false);

  RobotState           current_state_;
  StateChangeCallback  on_transition_cb_;
  uint32_t             total_fault_count_{0};
  std::chrono::steady_clock::time_point state_entry_time_;
};

}  // namespace state_machine
