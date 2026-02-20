/**
 * @file test_actuator_controller.cpp
 * @brief Unit tests for safety monitor (pure C++ logic, no ROS spin needed).
 */

#include <gtest/gtest.h>
#include "actuator_controller_cpp/safety_monitor.hpp"

using namespace actuator_controller;

class SafetyMonitorTest : public ::testing::Test
{
protected:
  SafetyLimits limits;
  std::unique_ptr<SafetyMonitor> monitor;

  void SetUp() override
  {
    limits.max_temperature_celsius   = 75.0;
    limits.emergency_stop_temperature = 85.0;
    limits.max_joint_velocity_rad_s  = 3.14;
    limits.max_torque_nm             = 50.0;
    limits.max_speed_fraction        = 1.0;
    limits.max_gripper_position      = 1.0;
    limits.min_gripper_position      = 0.0;
    monitor = std::make_unique<SafetyMonitor>(limits);
  }
};

TEST_F(SafetyMonitorTest, NominalTemperatureIsSafe)
{
  auto result = monitor->check_temperature(50.0);
  EXPECT_TRUE(result.safe);
  EXPECT_EQ(result.fault_code, 0);
}

TEST_F(SafetyMonitorTest, OverTemperatureIsUnsafe)
{
  auto result = monitor->check_temperature(80.0);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.fault_code, 1);  // FAULT_OVER_TEMPERATURE
}

TEST_F(SafetyMonitorTest, EmergencyStopTemperature)
{
  auto result = monitor->check_temperature(90.0);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.fault_code, 5);  // FAULT_EMERGENCY_STOP
}

TEST_F(SafetyMonitorTest, NominalVelocityIsSafe)
{
  std::vector<double> velocities = {1.0, 2.0, -1.5, 0.5, 0.0, 3.0};
  auto result = monitor->check_velocity_command(velocities);
  EXPECT_TRUE(result.safe);
}

TEST_F(SafetyMonitorTest, ExcessiveVelocityIsUnsafe)
{
  std::vector<double> velocities = {1.0, 5.0, 0.0};  // 5.0 > 3.14
  auto result = monitor->check_velocity_command(velocities);
  EXPECT_FALSE(result.safe);
}

TEST_F(SafetyMonitorTest, NominalTorqueIsSafe)
{
  std::vector<double> torques = {10.0, 20.0, 30.0};
  auto result = monitor->check_torque_command(torques);
  EXPECT_TRUE(result.safe);
}

TEST_F(SafetyMonitorTest, ExcessiveTorqueIsUnsafe)
{
  std::vector<double> torques = {10.0, 60.0};  // 60 > 50
  auto result = monitor->check_torque_command(torques);
  EXPECT_FALSE(result.safe);
}

TEST_F(SafetyMonitorTest, ValidGripperPositionIsSafe)
{
  EXPECT_TRUE(monitor->check_gripper_command(0.0).safe);
  EXPECT_TRUE(monitor->check_gripper_command(0.5).safe);
  EXPECT_TRUE(monitor->check_gripper_command(1.0).safe);
}

TEST_F(SafetyMonitorTest, InvalidGripperPositionIsUnsafe)
{
  EXPECT_FALSE(monitor->check_gripper_command(-0.1).safe);
  EXPECT_FALSE(monitor->check_gripper_command(1.1).safe);
  EXPECT_EQ(monitor->check_gripper_command(1.5).fault_code, 3);  // FAULT_POSITION_LIMIT
}

TEST_F(SafetyMonitorTest, ValidSpeedFractionIsSafe)
{
  EXPECT_TRUE(monitor->check_speed_fraction(0.0).safe);
  EXPECT_TRUE(monitor->check_speed_fraction(0.5).safe);
  EXPECT_TRUE(monitor->check_speed_fraction(1.0).safe);
}

TEST_F(SafetyMonitorTest, InvalidSpeedFractionIsUnsafe)
{
  EXPECT_FALSE(monitor->check_speed_fraction(-0.1).safe);
  EXPECT_FALSE(monitor->check_speed_fraction(1.5).safe);
}

TEST_F(SafetyMonitorTest, CheckAllReturnsFirstFailure)
{
  // Temperature violation + velocity violation — should return temperature fault
  std::vector<double> bad_velocities = {10.0};
  auto result = monitor->check_all(80.0, bad_velocities, {}, 0.5, 0.5);
  EXPECT_FALSE(result.safe);
  EXPECT_EQ(result.fault_code, 1);  // temperature checked first
}

TEST_F(SafetyMonitorTest, LimitsCanBeUpdatedAtRuntime)
{
  SafetyLimits new_limits = limits;
  new_limits.max_temperature_celsius = 50.0;
  monitor->update_limits(new_limits);

  auto result = monitor->check_temperature(60.0);
  EXPECT_FALSE(result.safe);  // was safe before update
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
