#include "odrive_protocol_policy.hpp"

#include <gtest/gtest.h>

TEST(ODriveProtocolPolicy, CachedHeartbeatCannotCompleteNewAxisRequest)
{
  EXPECT_FALSE(odrive_protocol_policy::axis_request_complete(
      41U, 41U, 3U, 1U, false, true));
  EXPECT_FALSE(odrive_protocol_policy::axis_request_complete(
      42U, 41U, 3U, 1U, false, false));
  EXPECT_TRUE(odrive_protocol_policy::axis_request_complete(
      42U, 41U, 3U, 1U, false, true));
  EXPECT_FALSE(odrive_protocol_policy::axis_request_complete(
      42U, 41U, 3U, 3U, true, true));
}

TEST(ODriveProtocolPolicy, UnrelatedCanTrafficCannotRefreshSafetyStatus)
{
  EXPECT_TRUE(odrive_protocol_policy::should_publish_controller_status(0x001U, 0x0FU));
  EXPECT_FALSE(odrive_protocol_policy::should_publish_controller_status(0x009U, 0x0FU));
  EXPECT_FALSE(odrive_protocol_policy::should_publish_controller_status(0x001U, 0x07U));

  EXPECT_TRUE(odrive_protocol_policy::should_publish_odrive_status(0x003U, 0x07U));
  EXPECT_FALSE(odrive_protocol_policy::should_publish_odrive_status(0x017U, 0x07U));
  EXPECT_FALSE(odrive_protocol_policy::should_publish_odrive_status(0x003U, 0x03U));
}

TEST(ODriveProtocolPolicy, EmergencyStopRequiresExactlyOneFreshReleasedAuthority)
{
  EXPECT_TRUE(odrive_protocol_policy::emergency_stop_released(
      false, 0U, false, true, 100.0, 0.5));

  EXPECT_TRUE(odrive_protocol_policy::emergency_stop_released(
      true, 1U, true, false, 0.5, 0.5));
  EXPECT_FALSE(odrive_protocol_policy::emergency_stop_released(
      true, 0U, true, false, 0.1, 0.5));
  EXPECT_FALSE(odrive_protocol_policy::emergency_stop_released(
      true, 2U, true, false, 0.1, 0.5));
  EXPECT_FALSE(odrive_protocol_policy::emergency_stop_released(
      true, 1U, false, false, 0.0, 0.5));
  EXPECT_FALSE(odrive_protocol_policy::emergency_stop_released(
      true, 1U, true, true, 0.1, 0.5));
  EXPECT_FALSE(odrive_protocol_policy::emergency_stop_released(
      true, 1U, true, false, 0.5001, 0.5));
}

TEST(ODriveProtocolPolicy, ProductionControlOwnerMustBeUnique)
{
  EXPECT_TRUE(odrive_protocol_policy::control_message_owner_valid(false, 0U));
  EXPECT_TRUE(odrive_protocol_policy::control_message_owner_valid(true, 1U));
  EXPECT_FALSE(odrive_protocol_policy::control_message_owner_valid(true, 0U));
  EXPECT_FALSE(odrive_protocol_policy::control_message_owner_valid(true, 2U));
}

TEST(ODriveProtocolPolicy, UnsafeStateAllowsOnlyIdleAxisRequest)
{
  constexpr uint32_t kIdle = 1U;
  constexpr uint32_t kFullCalibrationSequence = 3U;
  constexpr uint32_t kClosedLoopControl = 8U;

  EXPECT_TRUE(odrive_protocol_policy::axis_state_request_authorized(kIdle, false));
  EXPECT_FALSE(odrive_protocol_policy::axis_state_request_authorized(
      kFullCalibrationSequence, false));
  EXPECT_FALSE(odrive_protocol_policy::axis_state_request_authorized(
      kClosedLoopControl, false));
  EXPECT_TRUE(odrive_protocol_policy::axis_state_request_authorized(
      kFullCalibrationSequence, true));
  EXPECT_TRUE(odrive_protocol_policy::axis_state_request_authorized(
      kClosedLoopControl, true));
}

TEST(ODriveProtocolPolicy, UnsafeTransitionInvalidatesQueuedCommand)
{
  EXPECT_TRUE(odrive_protocol_policy::queued_command_authorized(9U, 9U, true));
  EXPECT_FALSE(odrive_protocol_policy::queued_command_authorized(9U, 10U, true));
  EXPECT_FALSE(odrive_protocol_policy::queued_command_authorized(9U, 9U, false));
}
