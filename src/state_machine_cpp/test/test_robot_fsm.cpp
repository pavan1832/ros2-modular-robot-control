/**
 * @file test_robot_fsm.cpp
 * @brief Comprehensive unit tests for RobotFsm.
 *
 * Tests cover all state transitions, guard conditions, fault counting,
 * and recovery flows without any ROS infrastructure.
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "state_machine_cpp/robot_fsm.hpp"

using namespace state_machine;

// ── Fixture ───────────────────────────────────────────────────────────────────

class RobotFsmTest : public ::testing::Test
{
protected:
  FsmContext healthy_ctx;
  FsmContext unhealthy_sensor_ctx;
  FsmContext unhealthy_actuator_ctx;

  void SetUp() override
  {
    healthy_ctx.sensor_healthy   = true;
    healthy_ctx.actuator_healthy = true;

    unhealthy_sensor_ctx.sensor_healthy   = false;
    unhealthy_sensor_ctx.actuator_healthy = true;

    unhealthy_actuator_ctx.sensor_healthy   = true;
    unhealthy_actuator_ctx.actuator_healthy = false;
  }

  RobotFsm make_fsm(RobotState initial = RobotState::IDLE)
  {
    return RobotFsm(initial);
  }
};

// ── Initial state ─────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, InitialStateIsIdle)
{
  auto fsm = make_fsm();
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);
  EXPECT_EQ(fsm.state_name(), "IDLE");
  EXPECT_EQ(fsm.total_fault_count(), 0u);
}

TEST_F(RobotFsmTest, InitialFaultCountIsZero)
{
  auto fsm = make_fsm();
  EXPECT_EQ(fsm.total_fault_count(), 0u);
}

// ── IDLE → ACTIVE ─────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, ActivateFromIdleWithHealthySystemSucceeds)
{
  auto fsm   = make_fsm();
  auto result = fsm.process_event(FsmEvent::ACTIVATE, healthy_ctx);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.new_state, RobotState::ACTIVE);
  EXPECT_EQ(fsm.current_state(), RobotState::ACTIVE);
}

TEST_F(RobotFsmTest, ActivateFromIdleWithUnhealthySensorFails)
{
  auto fsm    = make_fsm();
  auto result = fsm.process_event(FsmEvent::ACTIVATE, unhealthy_sensor_ctx);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);  // no transition
}

TEST_F(RobotFsmTest, ActivateFromIdleWithUnhealthyActuatorFails)
{
  auto fsm    = make_fsm();
  auto result = fsm.process_event(FsmEvent::ACTIVATE, unhealthy_actuator_ctx);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);
}

// ── ACTIVE → IDLE ─────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, DeactivateFromActiveSucceeds)
{
  auto fsm = make_fsm(RobotState::ACTIVE);
  auto result = fsm.process_event(FsmEvent::DEACTIVATE, healthy_ctx);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);
}

// ── ACTIVE → ERROR ────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, SensorFaultFromActiveGoesToError)
{
  auto fsm    = make_fsm(RobotState::ACTIVE);
  auto result = fsm.process_event(FsmEvent::SENSOR_FAULT, unhealthy_sensor_ctx);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
  EXPECT_EQ(fsm.total_fault_count(), 1u);
}

TEST_F(RobotFsmTest, ActuatorFaultFromActiveGoesToError)
{
  auto fsm    = make_fsm(RobotState::ACTIVE);
  auto result = fsm.process_event(FsmEvent::ACTUATOR_FAULT, healthy_ctx);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
  EXPECT_EQ(fsm.total_fault_count(), 1u);
}

TEST_F(RobotFsmTest, CommunicationFaultFromActiveGoesToError)
{
  auto fsm    = make_fsm(RobotState::ACTIVE);
  auto result = fsm.process_event(FsmEvent::COMMUNICATION_FAULT, healthy_ctx);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
  EXPECT_EQ(fsm.total_fault_count(), 1u);
}

// ── ERROR → RECOVERING → IDLE ─────────────────────────────────────────────────

TEST_F(RobotFsmTest, FullRecoveryFlowSucceeds)
{
  auto fsm = make_fsm(RobotState::ERROR);

  auto r1 = fsm.process_event(FsmEvent::BEGIN_RECOVERY);
  EXPECT_TRUE(r1.success);
  EXPECT_EQ(fsm.current_state(), RobotState::RECOVERING);

  auto r2 = fsm.process_event(FsmEvent::RECOVERED, healthy_ctx);
  EXPECT_TRUE(r2.success);
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);
}

TEST_F(RobotFsmTest, RecoveryFailsIfSubsystemsStillUnhealthy)
{
  auto fsm = make_fsm(RobotState::RECOVERING);

  auto result = fsm.process_event(FsmEvent::RECOVERED, unhealthy_sensor_ctx);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::RECOVERING);
}

TEST_F(RobotFsmTest, NewFaultDuringRecoveryGoesBackToError)
{
  auto fsm = make_fsm(RobotState::RECOVERING);

  auto result = fsm.process_event(FsmEvent::SENSOR_FAULT, unhealthy_sensor_ctx);
  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
  EXPECT_EQ(fsm.total_fault_count(), 1u);
}

// ── Emergency Stop ─────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, EmergencyStopFromIdleGoesToError)
{
  auto fsm    = make_fsm(RobotState::IDLE);
  auto result = fsm.process_event(FsmEvent::EMERGENCY_STOP);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
}

TEST_F(RobotFsmTest, EmergencyStopFromActiveGoesToError)
{
  auto fsm    = make_fsm(RobotState::ACTIVE);
  auto result = fsm.process_event(FsmEvent::EMERGENCY_STOP);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
  EXPECT_GE(fsm.total_fault_count(), 1u);
}

TEST_F(RobotFsmTest, EmergencyStopFromRecoveringGoesToError)
{
  auto fsm    = make_fsm(RobotState::RECOVERING);
  auto result = fsm.process_event(FsmEvent::EMERGENCY_STOP);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ERROR);
}

// ── Fault Counting ─────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, FaultCountAccumulatesAcrossMultipleFaults)
{
  auto fsm = make_fsm();

  // IDLE → ACTIVE
  fsm.process_event(FsmEvent::ACTIVATE, healthy_ctx);
  // ACTIVE → ERROR (fault 1)
  fsm.process_event(FsmEvent::SENSOR_FAULT);
  // ERROR → RECOVERING → IDLE
  fsm.process_event(FsmEvent::BEGIN_RECOVERY);
  fsm.process_event(FsmEvent::RECOVERED, healthy_ctx);
  // IDLE → ACTIVE
  fsm.process_event(FsmEvent::ACTIVATE, healthy_ctx);
  // ACTIVE → ERROR (fault 2)
  fsm.process_event(FsmEvent::ACTUATOR_FAULT);

  EXPECT_EQ(fsm.total_fault_count(), 2u);
}

// ── Transition Callback ────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, TransitionCallbackIsInvoked)
{
  std::vector<std::pair<RobotState, RobotState>> transitions;

  RobotFsm fsm(RobotState::IDLE,
    [&transitions](RobotState from, RobotState to, const std::string &) {
      transitions.emplace_back(from, to);
    });

  fsm.process_event(FsmEvent::ACTIVATE, healthy_ctx);   // IDLE → ACTIVE
  fsm.process_event(FsmEvent::SENSOR_FAULT);             // ACTIVE → ERROR

  ASSERT_EQ(transitions.size(), 2u);
  EXPECT_EQ(transitions[0].first,  RobotState::IDLE);
  EXPECT_EQ(transitions[0].second, RobotState::ACTIVE);
  EXPECT_EQ(transitions[1].first,  RobotState::ACTIVE);
  EXPECT_EQ(transitions[1].second, RobotState::ERROR);
}

// ── Time in state ──────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, TimeInStateIncreasesOverTime)
{
  auto fsm = make_fsm();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_GE(fsm.time_in_current_state_sec(), 0.04);
}

TEST_F(RobotFsmTest, TimeInStateResetsAfterTransition)
{
  auto fsm = make_fsm();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  fsm.process_event(FsmEvent::ACTIVATE, healthy_ctx);
  EXPECT_LT(fsm.time_in_current_state_sec(), 0.01);  // just transitioned
}

// ── Reset ─────────────────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, ResetReturnsToIdleAndClearsFaults)
{
  auto fsm = make_fsm(RobotState::ERROR);
  fsm.process_event(FsmEvent::EMERGENCY_STOP);  // would already be error

  fsm.reset();
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);
  EXPECT_EQ(fsm.total_fault_count(), 0u);
}

// ── Invalid transitions ────────────────────────────────────────────────────────

TEST_F(RobotFsmTest, InvalidEventInIdleDoesNotChangeState)
{
  auto fsm = make_fsm(RobotState::IDLE);
  auto result = fsm.process_event(FsmEvent::RECOVERED);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::IDLE);
}

TEST_F(RobotFsmTest, ActivateInActiveStateIsInvalid)
{
  auto fsm = make_fsm(RobotState::ACTIVE);
  auto result = fsm.process_event(FsmEvent::ACTIVATE, healthy_ctx);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(fsm.current_state(), RobotState::ACTIVE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
