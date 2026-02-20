/**
 * @file safety_monitor.cpp
 * @brief Implementation of hardware safety checks.
 */

#include "actuator_controller_cpp/safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace actuator_controller
{

SafetyMonitor::SafetyMonitor(const SafetyLimits & limits)
: limits_(limits)
{}

void SafetyMonitor::update_limits(const SafetyLimits & limits)
{
  limits_ = limits;
}

SafetyResult SafetyMonitor::check_temperature(double temperature_celsius) const
{
  if (temperature_celsius >= limits_.emergency_stop_temperature) {
    return {false, 5,  // FAULT_EMERGENCY_STOP
      "EMERGENCY STOP: temperature " + std::to_string(temperature_celsius) +
      "°C exceeds hard limit " + std::to_string(limits_.emergency_stop_temperature) + "°C"};
  }
  if (temperature_celsius >= limits_.max_temperature_celsius) {
    return {false, 1,  // FAULT_OVER_TEMPERATURE
      "Over-temperature: " + std::to_string(temperature_celsius) +
      "°C exceeds limit " + std::to_string(limits_.max_temperature_celsius) + "°C"};
  }
  return {true, 0, "temperature nominal"};
}

SafetyResult SafetyMonitor::check_velocity_command(const std::vector<double> & velocities) const
{
  for (size_t i = 0; i < velocities.size(); ++i) {
    if (std::abs(velocities[i]) > limits_.max_joint_velocity_rad_s) {
      std::ostringstream oss;
      oss << "Joint " << i << " velocity " << velocities[i]
          << " rad/s exceeds limit " << limits_.max_joint_velocity_rad_s << " rad/s";
      return {false, 2, oss.str()};  // FAULT_OVER_CURRENT (proxy for velocity limit)
    }
  }
  return {true, 0, "velocities nominal"};
}

SafetyResult SafetyMonitor::check_torque_command(const std::vector<double> & torques) const
{
  for (size_t i = 0; i < torques.size(); ++i) {
    if (std::abs(torques[i]) > limits_.max_torque_nm) {
      std::ostringstream oss;
      oss << "Joint " << i << " torque limit " << torques[i]
          << " N·m exceeds max " << limits_.max_torque_nm << " N·m";
      return {false, 2, oss.str()};
    }
  }
  return {true, 0, "torques nominal"};
}

SafetyResult SafetyMonitor::check_gripper_command(double gripper_position) const
{
  if (gripper_position < limits_.min_gripper_position ||
      gripper_position > limits_.max_gripper_position)
  {
    std::ostringstream oss;
    oss << "Gripper position " << gripper_position
        << " outside bounds [" << limits_.min_gripper_position
        << ", " << limits_.max_gripper_position << "]";
    return {false, 3, oss.str()};  // FAULT_POSITION_LIMIT
  }
  return {true, 0, "gripper nominal"};
}

SafetyResult SafetyMonitor::check_speed_fraction(double speed_fraction) const
{
  if (speed_fraction < 0.0 || speed_fraction > limits_.max_speed_fraction) {
    std::ostringstream oss;
    oss << "Speed fraction " << speed_fraction
        << " outside valid range [0, " << limits_.max_speed_fraction << "]";
    return {false, 3, oss.str()};
  }
  return {true, 0, "speed fraction nominal"};
}

SafetyResult SafetyMonitor::check_all(
  double temperature,
  const std::vector<double> & velocities,
  const std::vector<double> & torques,
  double gripper_position,
  double speed_fraction) const
{
  // Check in priority order: temperature first (most critical)
  for (auto & result : {
    check_temperature(temperature),
    check_velocity_command(velocities),
    check_torque_command(torques),
    check_gripper_command(gripper_position),
    check_speed_fraction(speed_fraction)
  }) {
    if (!result.safe) {
      return result;
    }
  }
  return {true, 0, "all checks nominal"};
}

}  // namespace actuator_controller
