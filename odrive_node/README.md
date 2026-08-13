# Standalone ODrive ROS2 node

This packages serves as a standalone ROS2 node to control the ODrive via CAN.

For information about installation, prerequisites and getting started, check out the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

## Interface

### Parameters

* `node_id`: The node_id of the device this node will attach to
* `interface`: the network interface name for the can bus
* `axis_idle_on_shutdown`: Whether to set ODrive to IDLE state when the node is terminated
* `axis_state_response_timeout_sec`: Maximum wait for a fresh completion heartbeat after an axis-state CAN request
* `can_send_timeout_sec`: Short transport-liveness bound for the ROS callback to event-loop CAN write handoff
* `emergency_stop_topic`: Lower-layer emergency stop heartbeat topic
* `require_emergency_stop`: Require exactly one fresh released heartbeat before any energizing command (default `false` for generic driver use)
* `emergency_stop_timeout_sec`: Emergency heartbeat freshness timeout
* `emergency_stop_check_period_sec`: Unsafe-state check and repeated IDLE request period
* `require_unique_control_message_publisher`: Require exactly one `/control_message` publisher (default `false`)

### Subscribes to

* `/control_message`: Input setpoints for the ODrive.

  The ODrive will interpret the values of input_pos, input_vel and input_torque depending on the control mode. 

  For example: In velocity control mode (2) input_pos is ignored, and input_torque is used as a feedforward term.

  **Note:** When changing `input_mode` or `control_mode`, it is advised to set the ODrive to IDLE before doing so. Changing these values during CLOSED_LOOP_CONTROL is not advised.

### Publishes

* `/odrive_status`: Provides ODrive/system level status updates.

  For this topic to work, the ODrive must be configured with the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages) enabled:

  - `error_msg_rate_ms`
  - `temperature_msg_rate_ms`
  - `bus_voltage_msg_rate_ms`

  The ROS node waits until one of each message has initialized the aggregate, then emits `odrive_status` only when a fresh `Get_Error` frame arrives. Unrelated traffic cannot refresh cached error state.

* `/controller_status`: Provides Controller level status updates. 

  For this topic to work, the ODrive must be configured with the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages) enabled:

  - `heartbeat_msg_rate_ms`
  - `encoder_msg_rate_ms`
  - `iq_msg_rate_ms`
  - `torques_msg_rate_ms`

  The ROS node waits until one of each message has initialized the aggregate, then emits `controller_status` only when a fresh heartbeat arrives. Unrelated traffic cannot refresh a cached axis/procedure state.

### Services

* `/request_axis_state`: Sets the axes requested state.

  This service requires regular heartbeat messages from the ODrive to determine the procedure result and will block until the procedure completes, with a minimum call time of 1 second. Completion always requires a heartbeat received after the matching CAN request was sent.

  This service does not clear errors implicitly. Fault acknowledgement is a separate, explicit `/clear_errors` operation.

* `/clear_errors`: Manual service call to clear disarm_reason and procedure_result, reset the LED color and re-arm the brake resistor if applicable. See also [`clear_errors()`](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.clear_errors).

  This does not affect the axis state.

  If the axis dropped into IDLE because of an error, call `/clear_errors` explicitly, verify fresh error-free status, and only then request CLOSED_LOOP_CONTROL.

* `/set_configs`: Sends one supported configuration CAN frame. The service response is returned only after the event-loop thread attempted the socket write; `success=false` means the frame was not sent. This is a transport acknowledgement, not an ODrive readback.

### Data Types

All of the Message/Service fields are directly related to their corresponding CAN message. For more detailed information about each type, and how to interpet the data, please refer to the [ODrive CAN protocol documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#messages).

Enum types and their corresponding integer values are listed in the ODrive API reference:

- [AxisState](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState)
- [ControlMode](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode)
- [InputMode](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode)

If you have the `odrive` Python package installed (not mandatory for this ROS node), you translate enums like this:

```py
from odrive.enums import AxisState
print(AxisState.CLOSED_LOOP_CONTROL) # 8
print(AxisState(8).name) # CLOSED_LOOP_CONTROL
```
