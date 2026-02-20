#pragma once

/**
 * @file safety_monitor.hpp
 * @brief Hardware safety enforcement layer for the actuator controller.
 *
 * SafetyMonitor is a pure C++ class (not a ROS node) so it can be unit
 * tested independently. It validates incoming commands and current actuator
 * state against configurable safety thresholds.
 */

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace actuator_controller
{

/// Safety check result
struct SafetyResult
{
  bool   safe{true};
  uint8_t fault_code{0};
  std::string message{};
};

/// Runtime-configurable safety limits
struct SafetyLimits
{
  double max_temperature_celsius{75.0};
  double max_joint_velocity_rad_s{3.14};   // π rad/s
  double max_torque_nm{50.0};
  double max_gripper_position{1.0};        // fully closed
  double min_gripper_position{0.0};        // fully open
  double max_speed_fraction{1.0};
  double emergency_stop_temperature{85.0}; // hard E-stop above this
};

/**
 * @class SafetyMonitor
 * @brief Validates actuator commands and state against safety thresholds.
 *
 * All methods are const-correct and side-effect free so they are safe
 * to call from any thread.
 */
class SafetyMonitor
{
public:
  explicit SafetyMonitor(const SafetyLimits & limits = SafetyLimits{});

  /// Update limits at runtime (e.g. from parameter callback)
  void update_limits(const SafetyLimits & limits);

  /// Check current actuator temperatures
  SafetyResult check_temperature(double temperature_celsius) const;

  /// Check commanded joint velocities
  SafetyResult check_velocity_command(const std::vector<double> & velocities) const;

  /// Check commanded torque limits
  SafetyResult check_torque_command(const std::vector<double> & torques) const;

  /// Check gripper position target [0, 1]
  SafetyResult check_gripper_command(double gripper_position) const;

  /// Check speed fraction [0, 1]
  SafetyResult check_speed_fraction(double speed_fraction) const;

  /// Aggregate all checks — returns first failed check or safe result
  SafetyResult check_all(
    double temperature,
    const std::vector<double> & velocities,
    const std::vector<double> & torques,
    double gripper_position,
    double speed_fraction) const;

  const SafetyLimits & limits() const noexcept { return limits_; }

private:
  SafetyLimits limits_;
};

}  // namespace actuator_controller
