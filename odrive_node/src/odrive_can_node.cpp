#include "odrive_can_node.hpp"

#include "byte_swap.hpp"
#include "epoll_event_loop.hpp"
#include "odrive_enums.h"
#include "odrive_protocol_policy.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sys/eventfd.h>

enum CmdId : uint32_t {
    kHeartbeat = 0x001, // ControllerStatus  - publisher
    kGetError = 0x003, // SystemStatus      - publisher
    kSetAxisState = 0x007, // SetAxisState      - service
    kGetEncoderEstimates = 0x009, // ControllerStatus  - publisher
    kSetControllerMode = 0x00b, // ControlMessage    - subscriber
    kSetInputPos, // ControlMessage    - subscriber
    kSetInputVel, // ControlMessage    - subscriber
    kSetInputTorque, // ControlMessage    - subscriber
    kGetIq = 0x014, // ControllerStatus  - publisher
    kGetTemp, // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
    kClearErrors = 0x018, // ClearErrors       - service
    kGetTorques = 0x01c, // ControllerStatus  - publisher

};

enum class ParamId : uint32_t {
    SetLimits = 0x00f, // Velocity_Limit, Current_Limit
    TrajVelLimit = 0x011, // Traj_Vel_Limit
    TrajAccelLimits = 0x012, // Traj_Accel_Limit, Traj_Decel_Limit
    TrajInertia = 0x013, // Traj_Inertia
    AbsolutePosition = 0x019, // Position
    PosGain = 0x01a, // Pos_Gain
    VelGains = 0x01b, // Vel_Gain, Vel_Integrator_Gain
};

static const std::unordered_map<std::string, ParamId> config_name_to_id = {
    {"traj_vel_limit", ParamId::TrajVelLimit},
    {"traj_accel_limit", ParamId::TrajAccelLimits},
    {"traj_inertia", ParamId::TrajInertia},
    {"pos_gain", ParamId::PosGain},
    {"vel_gains", ParamId::VelGains},
    {"absolute_position", ParamId::AbsolutePosition},
    {"set_limits", ParamId::SetLimits}
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);
    rclcpp::Node::declare_parameter<bool>("axis_idle_on_shutdown", false);
    rclcpp::Node::declare_parameter<double>("axis_state_response_timeout_sec", 100.0);
    rclcpp::Node::declare_parameter<double>("can_send_timeout_sec", 1.0);
    rclcpp::Node::declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
    rclcpp::Node::declare_parameter<bool>("require_emergency_stop", false);
    rclcpp::Node::declare_parameter<double>("emergency_stop_timeout_sec", 0.5);
    rclcpp::Node::declare_parameter<double>("emergency_stop_check_period_sec", 0.05);
    rclcpp::Node::declare_parameter<bool>("require_unique_control_message_publisher", false);

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepAll{});
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);

    rclcpp::QoS odrv_stat_qos(rclcpp::KeepAll{});
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>(
        "control_message",
        ctrl_msg_qos,
        std::bind(&ODriveCanNode::subscriber_callback, this, _1)
    );

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});
    service_ = rclcpp::Node::create_service<AxisState>(
        "request_axis_state",
        std::bind(&ODriveCanNode::service_callback, this, _1, _2),
        srv_qos
    );

    rclcpp::QoS srv_clear_errors_qos(rclcpp::KeepAll{});
    service_clear_errors_ = rclcpp::Node::create_service<Empty>(
        "clear_errors",
        std::bind(&ODriveCanNode::service_clear_errors_callback, this, _1, _2),
        srv_clear_errors_qos
    );

    rclcpp::QoS srv_set_configs_qos(rclcpp::KeepAll{});
    service_set_configs_ = rclcpp::Node::create_service<SetConfigs>(
        "set_configs",
        std::bind(&ODriveCanNode::service_set_configs_callback, this, _1, _2),
        srv_set_configs_qos
    );
}

void ODriveCanNode::deinit() {
    if (safety_watchdog_timer_) {
        safety_watchdog_timer_->cancel();
    }
    emergency_stop_subscriber_.reset();

    // Stop and drain every producer of outgoing commands first. The event-loop
    // registration lock makes this wait for an in-flight callback, so no
    // queued state/config command can be sent after the final IDLE below.
    sub_evt_.deinit();
    srv_evt_.deinit();
    srv_clear_errors_evt_.deinit();
    srv_set_configs_evt_.deinit();
    if (safety_resources_initialized_) {
        safety_evt_.deinit();
        safety_resources_initialized_ = false;
    }

    if (axis_idle_on_shutdown_) {
        struct can_frame frame {};
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
        frame.can_dlc = 4;
        if (!can_intf_.send_can_frame(frame)) {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to send shutdown IDLE request");
        }
    }

    // Removing the socket is the final registration and wakes the event loop,
    // allowing main() to join it before the node is destroyed.
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {
    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    axis_idle_on_shutdown_ = rclcpp::Node::get_parameter("axis_idle_on_shutdown").as_bool();
    axis_state_response_timeout_sec_ =
        rclcpp::Node::get_parameter("axis_state_response_timeout_sec").as_double();
    if (!std::isfinite(axis_state_response_timeout_sec_) || axis_state_response_timeout_sec_ <= 1.0) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "axis_state_response_timeout_sec must be finite and greater than 1 second");
        return false;
    }
    can_send_timeout_sec_ =
        rclcpp::Node::get_parameter("can_send_timeout_sec").as_double();
    if (!std::isfinite(can_send_timeout_sec_) || can_send_timeout_sec_ <= 0.0) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "can_send_timeout_sec must be positive and finite");
        return false;
    }
    emergency_stop_topic_ =
        rclcpp::Node::get_parameter("emergency_stop_topic").as_string();
    require_emergency_stop_ =
        rclcpp::Node::get_parameter("require_emergency_stop").as_bool();
    emergency_stop_timeout_sec_ =
        rclcpp::Node::get_parameter("emergency_stop_timeout_sec").as_double();
    emergency_stop_check_period_sec_ =
        rclcpp::Node::get_parameter("emergency_stop_check_period_sec").as_double();
    require_unique_control_message_publisher_ = rclcpp::Node::get_parameter(
        "require_unique_control_message_publisher").as_bool();
    if (require_emergency_stop_ && emergency_stop_topic_.empty()) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "emergency_stop_topic must not be empty when require_emergency_stop is true");
        return false;
    }
    if (!std::isfinite(emergency_stop_timeout_sec_) || emergency_stop_timeout_sec_ <= 0.0) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "emergency_stop_timeout_sec must be positive and finite");
        return false;
    }
    if (!std::isfinite(emergency_stop_check_period_sec_) ||
        emergency_stop_check_period_sec_ <= 0.0 ||
        emergency_stop_check_period_sec_ >= emergency_stop_timeout_sec_) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "emergency_stop_check_period_sec must be positive, finite, and shorter than the timeout");
        return false;
    }
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        return false;
    }
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        return false;
    }
    if (!srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
        return false;
    }
    if (!srv_clear_errors_evt_.init(event_loop, std::bind(&ODriveCanNode::request_clear_errors_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
        return false;
    }
    if (!srv_set_configs_evt_.init(event_loop, std::bind(&ODriveCanNode::request_set_configs_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize set configs service event");
        return false;
    }
    if (!safety_evt_.init(event_loop, std::bind(&ODriveCanNode::safety_idle_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize safety IDLE event");
        return false;
    }
    safety_resources_initialized_ = true;

    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        commands_authorized_ = !require_emergency_stop_ &&
            !require_unique_control_message_publisher_;
        commands_authorized_snapshot_.store(commands_authorized_);
        authorization_generation_snapshot_.store(authorization_generation_);
    }

    if (require_emergency_stop_) {
        safety_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = safety_callback_group_;
        emergency_stop_subscriber_ = create_subscription<std_msgs::msg::Bool>(
            emergency_stop_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            std::bind(&ODriveCanNode::emergency_stop_callback, this, _1),
            subscription_options);
    }
    if (require_emergency_stop_ || require_unique_control_message_publisher_) {
        if (!safety_callback_group_) {
            safety_callback_group_ = create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
        }
        const auto watchdog_period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(emergency_stop_check_period_sec_));
        safety_watchdog_timer_ = create_wall_timer(
            watchdog_period,
            std::bind(&ODriveCanNode::safety_watchdog_callback, this),
            safety_callback_group_);
        // Start fail-closed before ROS discovery or the first heartbeat.
        safety_watchdog_callback();
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    RCLCPP_INFO(
        rclcpp::Node::get_logger(),
        "lower safety gate: emergency_stop_required=%s topic=%s timeout=%.3f s check=%.3f s unique_control_publisher=%s",
        require_emergency_stop_ ? "true" : "false", emergency_stop_topic_.c_str(),
        emergency_stop_timeout_sec_, emergency_stop_check_period_sec_,
        require_unique_control_message_publisher_ ? "true" : "false");
    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {
    if (((frame.can_id >> 5) & 0x3F) != node_id_)
        return;

    const auto command_id = frame.can_id & 0x1F;
    switch (command_id) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            ctrl_pub_flag_ |= 0b0001;
            ++heartbeat_generation_;
            last_heartbeat_time_ = std::chrono::steady_clock::now();
            fresh_heartbeat_.notify_all();
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc))
                break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000;
            break;
        }
        case CmdId::kSetAxisState:
        case CmdId::kSetControllerMode:
        case CmdId::kSetInputPos:
        case CmdId::kSetInputVel:
        case CmdId::kSetInputTorque:
        case CmdId::kClearErrors: {
            break; // Ignore commands coming from another master/host on the bus
        }
        default: {
            // RCLCPP_WARN(
            //   rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }

    // Heartbeat and GetError are the liveness authorities. Wait for all
    // documented fields to be initialized once, then publish ControllerStatus
    // only on a fresh heartbeat and ODriveStatus only on a fresh GetError.
    // Unrelated cyclic traffic must never refresh cached safety state.
    std::optional<ControllerStatus> controller_status;
    {
        std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
        if (odrive_protocol_policy::should_publish_controller_status(
                command_id, static_cast<uint8_t>(ctrl_pub_flag_))) {
            controller_status = ctrl_stat_;
        }
    }
    if (controller_status) {
        ctrl_publisher_->publish(*controller_status);
    }

    std::optional<ODriveStatus> odrive_status;
    {
        std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
        if (odrive_protocol_policy::should_publish_odrive_status(
                command_id, static_cast<uint8_t>(odrv_pub_flag_))) {
            odrive_status = odrv_stat_;
        }
    }
    if (odrive_status) {
        odrv_publisher_->publish(*odrive_status);
    }
}

void ODriveCanNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    bool authorized;
    bool authorization_changed;
    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        emergency_stop_received_ = true;
        emergency_stop_active_ = msg->data;
        emergency_stop_updated_at_ = now;
        authorization_changed = refresh_command_authorization_locked(
            now,
            emergency_stop_subscriber_ ? emergency_stop_subscriber_->get_publisher_count() : 0U,
            subscriber_ ? subscriber_->get_publisher_count() : 0U);
        authorized = commands_authorized_;
    }
    if (authorization_changed || !authorized) {
        // Pair the safety-generation transition with the mutex used by the
        // AxisState condition variable so the wakeup cannot be lost between
        // predicate evaluation and blocking.
        std::lock_guard<std::mutex> status_guard(ctrl_stat_mutex_);
        fresh_heartbeat_.notify_all();
    }
    if (!authorized) {
        request_safety_idle();
    }
}

void ODriveCanNode::safety_watchdog_callback() {
    const auto now = std::chrono::steady_clock::now();
    bool authorized;
    bool authorization_changed;
    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        authorization_changed = refresh_command_authorization_locked(
            now,
            emergency_stop_subscriber_ ? emergency_stop_subscriber_->get_publisher_count() : 0U,
            subscriber_ ? subscriber_->get_publisher_count() : 0U);
        authorized = commands_authorized_;
    }
    if (authorization_changed || !authorized) {
        std::lock_guard<std::mutex> status_guard(ctrl_stat_mutex_);
        fresh_heartbeat_.notify_all();
    }
    // Continue requesting IDLE at the configured check period for as long as
    // the lower-layer gate is unsafe. This also covers a lost upper manager.
    if (!authorized) {
        request_safety_idle();
    }
}

bool ODriveCanNode::refresh_command_authorization_locked(
    std::chrono::steady_clock::time_point now,
    std::size_t emergency_stop_publisher_count,
    std::size_t control_message_publisher_count
) {
    emergency_stop_publisher_count_ = emergency_stop_publisher_count;
    control_message_publisher_count_ = control_message_publisher_count;
    const double sample_age_sec = emergency_stop_received_ ?
        std::chrono::duration<double>(now - emergency_stop_updated_at_).count() :
        std::numeric_limits<double>::infinity();
    const bool emergency_stop_released =
        odrive_protocol_policy::emergency_stop_released(
            require_emergency_stop_, emergency_stop_publisher_count_,
            emergency_stop_received_, emergency_stop_active_, sample_age_sec,
            emergency_stop_timeout_sec_);
    const bool control_owner_valid =
        odrive_protocol_policy::control_message_owner_valid(
            require_unique_control_message_publisher_,
            control_message_publisher_count_);
    const bool authorized = odrive_protocol_policy::command_authorized(
        emergency_stop_released, control_owner_valid);
    if (authorized == commands_authorized_) {
        return false;
    }

    commands_authorized_ = authorized;
    ++authorization_generation_;
    commands_authorized_snapshot_.store(authorized);
    authorization_generation_snapshot_.store(authorization_generation_);
    if (authorized) {
        RCLCPP_INFO(
            rclcpp::Node::get_logger(),
            "lower ODrive command gate released (E-stop publishers=%zu, control publishers=%zu)",
            emergency_stop_publisher_count_, control_message_publisher_count_);
    } else {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "lower ODrive command gate unsafe; forcing IDLE (E-stop publishers=%zu received=%s active=%s age=%.3f s, control publishers=%zu)",
            emergency_stop_publisher_count_, emergency_stop_received_ ? "true" : "false",
            emergency_stop_active_ ? "true" : "false", sample_age_sec,
            control_message_publisher_count_);
    }
    return true;
}

bool ODriveCanNode::command_authorized_locked() const {
    return commands_authorized_;
}

void ODriveCanNode::request_safety_idle() {
    if (safety_resources_initialized_ && !safety_evt_.set()) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "failed to enqueue lower ODrive safety IDLE request");
    }
}

void ODriveCanNode::safety_idle_callback() {
    struct can_frame frame {};
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
    frame.can_dlc = 4;
    if (!can_intf_.send_can_frame(frame)) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to send safety IDLE CAN request");
    }
}

void ODriveCanNode::set_fail_closed_axis_response(
    std::shared_ptr<AxisState::Response> response
) const {
    response->axis_state = ODriveAxisState::AXIS_STATE_UNDEFINED;
    response->active_errors = std::numeric_limits<uint32_t>::max();
    response->procedure_result = ODriveProcedureResult::PROCEDURE_RESULT_TIMEOUT;
}

void ODriveCanNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
    safety_watchdog_callback();
    bool authorized;
    uint64_t generation;
    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        authorized = command_authorized_locked();
        generation = authorization_generation_;
        if (authorized) {
            // Commit the message payload and generation while the safety
            // generation is stable. An E-stop transition cannot slip between
            // authorization and the event-loop handoff metadata.
            std::lock_guard<std::mutex> control_guard(ctrl_msg_mutex_);
            ctrl_msg_ = *msg;
            ctrl_msg_authorization_generation_ = generation;
        }
    }
    if (!authorized) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "rejected control_message while lower ODrive command gate is unsafe");
        request_safety_idle();
        return;
    }
    if (!sub_evt_.set()) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to enqueue control_message CAN request");
        request_safety_idle();
    }
}

void ODriveCanNode::service_callback(
    const std::shared_ptr<AxisState::Request> request,
    std::shared_ptr<AxisState::Response> response
) {
    // MultiThreadedExecutor keeps the independent safety callback group alive
    // during this physical wait. Serialize state procedures themselves so two
    // requests can never share one completion heartbeat.
    std::unique_lock<std::mutex> service_guard(axis_service_mutex_);

    safety_watchdog_callback();
    bool commands_authorized;
    uint64_t authorization_generation;
    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        commands_authorized = command_authorized_locked();
        authorization_generation = authorization_generation_;
    }
    if (!odrive_protocol_policy::axis_state_request_authorized(
            request->axis_requested_state, commands_authorized)) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "rejected non-IDLE axis state %u while lower ODrive command gate is unsafe",
            request->axis_requested_state);
        request_safety_idle();
        set_fail_closed_axis_response(response);
        return;
    }

    auto pending = std::make_shared<PendingAxisSend>();
    pending->axis_state = request->axis_requested_state;
    pending->authorization_generation_at_enqueue = authorization_generation;
    {
        std::lock_guard<std::mutex> guard(axis_request_queue_mutex_);
        axis_request_queue_.push_back(pending);
    }
    RCLCPP_INFO(
        rclcpp::Node::get_logger(),
        "requesting axis state: %d",
        pending->axis_state);
    if (!srv_evt_.set()) {
        {
            std::lock_guard<std::mutex> guard(axis_request_queue_mutex_);
            auto it = std::find(axis_request_queue_.begin(), axis_request_queue_.end(), pending);
            if (it != axis_request_queue_.end()) {
                axis_request_queue_.erase(it);
            }
        }
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to enqueue axis state CAN request");
        set_fail_closed_axis_response(response);
        return;
    }

    uint64_t heartbeat_generation_at_send;
    std::chrono::steady_clock::time_point request_sent_time;
    {
        std::unique_lock<std::mutex> guard(pending->mutex);
        const bool send_completed = pending->completed.wait_for(
            guard,
            std::chrono::duration<double>(can_send_timeout_sec_),
            [&pending]() { return pending->done; });
        if (!send_completed) {
            pending->cancelled = true;
        }
        if (!send_completed || !pending->success) {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to send axis state CAN request");
            set_fail_closed_axis_response(response);
            return;
        }
        heartbeat_generation_at_send = pending->heartbeat_generation_at_send;
        request_sent_time = pending->sent_time;
    }

    // A response is valid only after the event-loop thread has actually sent
    // this request and at least one newer heartbeat has arrived. Without this
    // generation barrier, a cached SUCCESS heartbeat can complete a new
    // calibration request before the CAN request even leaves this process.
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_);
    const auto response_deadline = request_sent_time +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(axis_state_response_timeout_sec_));
    const bool completed = fresh_heartbeat_.wait_until(
        guard,
        response_deadline,
        [this, heartbeat_generation_at_send, request_sent_time,
         authorization_generation, &request]() {
        if (request->axis_requested_state != ODriveAxisState::AXIS_STATE_IDLE &&
            (!commands_authorized_snapshot_.load() ||
             authorization_generation_snapshot_.load() != authorization_generation)) {
            return true;
        }
        bool is_busy = this->ctrl_stat_.procedure_result == ODriveProcedureResult::PROCEDURE_RESULT_BUSY;
        bool heartbeat_after_minimum_time =
            (last_heartbeat_time_ - request_sent_time >= std::chrono::seconds(1));
        return odrive_protocol_policy::axis_request_complete(
            heartbeat_generation_,
            heartbeat_generation_at_send,
            request->axis_requested_state,
            ctrl_stat_.axis_state,
            is_busy,
            heartbeat_after_minimum_time);
    });

    const bool safety_interrupted =
        request->axis_requested_state != ODriveAxisState::AXIS_STATE_IDLE &&
        (!commands_authorized_snapshot_.load() ||
         authorization_generation_snapshot_.load() != authorization_generation);
    if (!completed || safety_interrupted) {
        if (safety_interrupted) {
            RCLCPP_ERROR(
                rclcpp::Node::get_logger(),
                "axis state request interrupted by lower ODrive safety gate");
        } else {
            RCLCPP_ERROR(
                rclcpp::Node::get_logger(),
                "axis state request timed out after %.3f seconds without a fresh completion heartbeat",
                axis_state_response_timeout_sec_);
        }
        // AxisState.srv predates an explicit success flag. Return unmistakable
        // fail-closed sentinels so every existing caller rejects the response.
        set_fail_closed_axis_response(response);
        return;
    }

    response->axis_state = ctrl_stat_.axis_state;
    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}

void ODriveCanNode::service_clear_errors_callback(
    const std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response
) {
    (void)request;
    (void)response;
    safety_watchdog_callback();
    uint64_t authorization_generation;
    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        if (!command_authorized_locked()) {
            RCLCPP_ERROR(
                rclcpp::Node::get_logger(),
                "rejected clear_errors while lower ODrive command gate is unsafe");
            request_safety_idle();
            return;
        }
        authorization_generation = authorization_generation_;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
    auto pending = std::make_shared<PendingCanSend>();
    pending->authorization_generation_at_enqueue = authorization_generation;
    {
        std::lock_guard<std::mutex> guard(clear_errors_queue_mutex_);
        clear_errors_queue_.push_back(pending);
    }
    if (!srv_clear_errors_evt_.set()) {
        {
            std::lock_guard<std::mutex> guard(clear_errors_queue_mutex_);
            auto it = std::find(clear_errors_queue_.begin(), clear_errors_queue_.end(), pending);
            if (it != clear_errors_queue_.end()) {
                clear_errors_queue_.erase(it);
            }
        }
        {
            std::lock_guard<std::mutex> guard(pending->mutex);
            pending->done = true;
            pending->success = false;
        }
    }

    std::unique_lock<std::mutex> guard(pending->mutex);
    const bool send_completed = pending->completed.wait_for(
        guard,
        std::chrono::duration<double>(can_send_timeout_sec_),
        [&pending]() { return pending->done; });
    if (!send_completed) {
        pending->cancelled = true;
    }
    if (!send_completed || !pending->success) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to send clear_errors CAN request");
    }
}

void ODriveCanNode::service_set_configs_callback(
    const std::shared_ptr<SetConfigs::Request> request,
    std::shared_ptr<SetConfigs::Response> response
) {
    safety_watchdog_callback();
    uint64_t authorization_generation;
    {
        std::lock_guard<std::mutex> guard(safety_mutex_);
        if (!command_authorized_locked()) {
            RCLCPP_ERROR(
                rclcpp::Node::get_logger(),
                "rejected set_configs while lower ODrive command gate is unsafe");
            response->success = false;
            response->message = "lower ODrive safety gate is unsafe";
            request_safety_idle();
            return;
        }
        authorization_generation = authorization_generation_;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "setting config: %s = %f", request->param_name.c_str(), request->value);
    auto it = config_name_to_id.find(request->param_name);
    if (it == config_name_to_id.end()) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Unknown config name: %s", request->param_name.c_str());
        response->success = false;
        response->message = "unknown config name";
        return;
    }
    if (!std::isfinite(request->value)) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Config value must be finite");
        response->success = false;
        response->message = "config value must be finite";
        return;
    }

    auto pending = std::make_shared<PendingConfigSend>();
    pending->param_name = request->param_name;
    pending->value = request->value;
    pending->authorization_generation_at_enqueue = authorization_generation;
    {
        std::lock_guard<std::mutex> guard(set_configs_queue_mutex_);
        set_configs_queue_.push_back(pending);
    }
    if (!srv_set_configs_evt_.set()) {
        {
            std::lock_guard<std::mutex> guard(set_configs_queue_mutex_);
            auto it = std::find(set_configs_queue_.begin(), set_configs_queue_.end(), pending);
            if (it != set_configs_queue_.end()) {
                set_configs_queue_.erase(it);
            }
        }
        {
            std::lock_guard<std::mutex> guard(pending->mutex);
            pending->done = true;
            pending->success = false;
        }
    }

    std::unique_lock<std::mutex> guard(pending->mutex);
    const bool send_completed = pending->completed.wait_for(
        guard,
        std::chrono::duration<double>(can_send_timeout_sec_),
        [&pending]() { return pending->done; });
    if (!send_completed) {
        pending->cancelled = true;
    }
    response->success = send_completed && pending->success;
    response->message = pending->success ?
        "CAN config frame sent" : "failed to send CAN config frame";
}

void ODriveCanNode::request_state_callback() {
    while (true) {
        std::shared_ptr<PendingAxisSend> pending;
        {
            std::lock_guard<std::mutex> guard(axis_request_queue_mutex_);
            if (axis_request_queue_.empty()) {
                return;
            }
            pending = axis_request_queue_.front();
            axis_request_queue_.pop_front();
        }

        {
            std::lock_guard<std::mutex> guard(pending->mutex);
            if (!pending->cancelled) {
                const auto current_generation = authorization_generation_snapshot_.load();
                const bool current_authorized = commands_authorized_snapshot_.load();
                const bool idle_request =
                    pending->axis_state == ODriveAxisState::AXIS_STATE_IDLE;
                if (!idle_request &&
                    !odrive_protocol_policy::queued_command_authorized(
                        pending->authorization_generation_at_enqueue,
                        current_generation, current_authorized)) {
                    RCLCPP_ERROR(
                        rclcpp::Node::get_logger(),
                        "dropped queued non-IDLE axis state after safety transition");
                    pending->success = false;
                    pending->done = true;
                    pending->completed.notify_one();
                    continue;
                }
                struct can_frame frame {};
                frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
                write_le<uint32_t>(pending->axis_state, frame.data);
                frame.can_dlc = 4;
                pending->success = can_intf_.send_can_frame(frame);
                pending->sent_time = std::chrono::steady_clock::now();
                std::lock_guard<std::mutex> status_guard(ctrl_stat_mutex_);
                pending->heartbeat_generation_at_send = heartbeat_generation_;
            }
            pending->done = true;
        }
        pending->completed.notify_one();
    }
}

void ODriveCanNode::request_clear_errors_callback() {
    while (true) {
        std::shared_ptr<PendingCanSend> pending;
        {
            std::lock_guard<std::mutex> guard(clear_errors_queue_mutex_);
            if (clear_errors_queue_.empty()) {
                return;
            }
            pending = clear_errors_queue_.front();
            clear_errors_queue_.pop_front();
        }

        {
            std::lock_guard<std::mutex> guard(pending->mutex);
            if (!pending->cancelled) {
                if (!odrive_protocol_policy::queued_command_authorized(
                        pending->authorization_generation_at_enqueue,
                        authorization_generation_snapshot_.load(),
                        commands_authorized_snapshot_.load())) {
                    RCLCPP_ERROR(
                        rclcpp::Node::get_logger(),
                        "dropped queued clear_errors after safety transition");
                    pending->success = false;
                    pending->done = true;
                    pending->completed.notify_one();
                    continue;
                }
                struct can_frame frame {};
                frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
                write_le<uint8_t>(0, frame.data);
                frame.can_dlc = 1;
                pending->success = can_intf_.send_can_frame(frame);
            }
            pending->done = true;
        }
        pending->completed.notify_one();
    }
}

void ODriveCanNode::request_set_configs_callback() {
    // eventfd notifications may coalesce, so drain every queued request.
    while (true) {
        std::shared_ptr<PendingConfigSend> pending;
        {
            std::lock_guard<std::mutex> guard(set_configs_queue_mutex_);
            if (set_configs_queue_.empty()) {
                return;
            }
            pending = set_configs_queue_.front();
            set_configs_queue_.pop_front();
        }

        {
            std::lock_guard<std::mutex> guard(pending->mutex);
            auto it = config_name_to_id.find(pending->param_name);
            if (!pending->cancelled && it != config_name_to_id.end()) {
                if (!odrive_protocol_policy::queued_command_authorized(
                        pending->authorization_generation_at_enqueue,
                        authorization_generation_snapshot_.load(),
                        commands_authorized_snapshot_.load())) {
                    RCLCPP_ERROR(
                        rclcpp::Node::get_logger(),
                        "dropped queued set_configs after safety transition");
                    pending->success = false;
                    pending->done = true;
                    pending->completed.notify_one();
                    continue;
                }
                struct can_frame frame {};
                frame.can_id = (node_id_ << 5) | static_cast<uint32_t>(it->second);

                if (it->second == ParamId::TrajAccelLimits) {
                    // ODrive CAN Set_Traj_Accel_Limits uses accel/decel floats.
                    write_le<float>(pending->value, frame.data + 0);
                    write_le<float>(pending->value, frame.data + 4);
                    frame.can_dlc = 8;
                } else {
                    write_le<float>(pending->value, frame.data);
                    frame.can_dlc = 4;
                }

                pending->success = can_intf_.send_can_frame(frame);
                if (pending->success) {
                    RCLCPP_INFO(
                        rclcpp::Node::get_logger(),
                        "Sent config: %s (id: 0x%x) = %f",
                        pending->param_name.c_str(),
                        static_cast<uint32_t>(it->second),
                        pending->value);
                }
            }
            pending->done = true;
        }
        pending->completed.notify_one();
    }
}

void ODriveCanNode::ctrl_msg_callback() {
    uint32_t control_mode;
    uint64_t authorization_generation;
    struct can_frame frame {};
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode, frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
        authorization_generation = ctrl_msg_authorization_generation_;
    }
    if (!odrive_protocol_policy::queued_command_authorized(
            authorization_generation,
            authorization_generation_snapshot_.load(),
            commands_authorized_snapshot_.load())) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "dropped queued control_message after safety transition");
        return;
    }
    frame.can_dlc = 8;
    if (!can_intf_.send_can_frame(frame)) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to send Set_Controller_Mode");
        return;
    }

    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel, frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos, frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)), frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }
        default: RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode); return;
    }

    // No callback can interleave between these two writes because all CAN
    // output is owned by the epoll event-loop thread. Revalidate immediately
    // before the physical setpoint write in case the safety generation changed
    // while the first frame was being sent.
    if (!odrive_protocol_policy::queued_command_authorized(
            authorization_generation,
            authorization_generation_snapshot_.load(),
            commands_authorized_snapshot_.load())) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "dropped control setpoint after safety transition");
        return;
    }
    if (!can_intf_.send_can_frame(frame)) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "failed to send control setpoint");
    }
}

inline bool ODriveCanNode::verify_length(const std::string& name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid)
        RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}
