#ifndef ODRIVE_PROTOCOL_POLICY_HPP
#define ODRIVE_PROTOCOL_POLICY_HPP

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace odrive_protocol_policy
{

inline bool should_publish_controller_status(uint32_t command_id, uint8_t initialized_fields)
{
  return command_id == 0x001U && initialized_fields == 0x0FU;
}

inline bool should_publish_odrive_status(uint32_t command_id, uint8_t initialized_fields)
{
  return command_id == 0x003U && initialized_fields == 0x07U;
}

inline bool axis_request_complete(
  uint64_t heartbeat_generation,
  uint64_t heartbeat_generation_at_send,
  uint32_t requested_axis_state,
  uint8_t observed_axis_state,
  bool procedure_busy,
  bool heartbeat_after_minimum_time)
{
  if (heartbeat_generation <= heartbeat_generation_at_send || !heartbeat_after_minimum_time) {
    return false;
  }

  constexpr uint32_t kIdle = 1U;
  constexpr uint32_t kClosedLoopControl = 8U;
  if (requested_axis_state == kClosedLoopControl) {
    return observed_axis_state == kClosedLoopControl;
  }

  return observed_axis_state == kIdle && !procedure_busy;
}

inline bool emergency_stop_released(
  bool required,
  std::size_t publisher_count,
  bool sample_received,
  bool stop_active,
  double sample_age_sec,
  double timeout_sec)
{
  if (!required) {
    return true;
  }
  return publisher_count == 1U && sample_received && !stop_active &&
         std::isfinite(sample_age_sec) && sample_age_sec >= 0.0 &&
         std::isfinite(timeout_sec) && timeout_sec > 0.0 &&
         sample_age_sec <= timeout_sec;
}

inline bool control_message_owner_valid(bool required, std::size_t publisher_count)
{
  return !required || publisher_count == 1U;
}

inline bool command_authorized(bool emergency_stop_released, bool control_owner_valid)
{
  return emergency_stop_released && control_owner_valid;
}

inline bool axis_state_request_authorized(uint32_t requested_axis_state, bool commands_authorized)
{
  constexpr uint32_t kIdle = 1U;
  return requested_axis_state == kIdle || commands_authorized;
}

inline bool queued_command_authorized(
  uint64_t authorization_generation_at_enqueue,
  uint64_t current_authorization_generation,
  bool commands_authorized)
{
  return commands_authorized &&
         authorization_generation_at_enqueue == current_authorization_generation;
}

}  // namespace odrive_protocol_policy

#endif  // ODRIVE_PROTOCOL_POLICY_HPP
