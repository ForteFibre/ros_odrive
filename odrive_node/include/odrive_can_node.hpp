#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "odrive_can/srv/set_configs.hpp"
#include "socket_can.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

using AxisState = odrive_can::srv::AxisState;
using Empty = std_srvs::srv::Empty;
using SetConfigs = odrive_can::srv::SetConfigs;

class ODriveCanNode : public rclcpp::Node {
public:
    ODriveCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop);
    void deinit();

private:
    struct PendingCanSend {
        std::mutex mutex;
        std::condition_variable completed;
        bool done{false};
        bool success{false};
        bool cancelled{false};
        uint64_t authorization_generation_at_enqueue{0};
    };

    struct PendingConfigSend : PendingCanSend {
        std::string param_name;
        float value{0.0F};
    };

    struct PendingAxisSend : PendingCanSend {
        uint32_t axis_state{0};
        uint64_t heartbeat_generation_at_send{0};
        std::chrono::steady_clock::time_point sent_time{};
    };

    void recv_callback(const can_frame& frame);
    void subscriber_callback(const ControlMessage::SharedPtr msg);
    void service_callback(
        const std::shared_ptr<AxisState::Request> request,
        std::shared_ptr<AxisState::Response> response
    );
    void service_clear_errors_callback(
        const std::shared_ptr<Empty::Request> request,
        std::shared_ptr<Empty::Response> response
    );
    void service_set_configs_callback(
        const std::shared_ptr<SetConfigs::Request> request,
        std::shared_ptr<SetConfigs::Response> response
    );
    void request_state_callback();
    void request_clear_errors_callback();
    void request_set_configs_callback();
    void ctrl_msg_callback();
    void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void safety_watchdog_callback();
    void safety_idle_callback();
    bool refresh_command_authorization_locked(
        std::chrono::steady_clock::time_point now,
        std::size_t emergency_stop_publisher_count,
        std::size_t control_message_publisher_count);
    bool command_authorized_locked() const;
    void request_safety_idle();
    void set_fail_closed_axis_response(std::shared_ptr<AxisState::Response> response) const;
    inline bool verify_length(const std::string& name, uint8_t expected, uint8_t length);

    uint16_t node_id_;
    bool axis_idle_on_shutdown_;
    double axis_state_response_timeout_sec_;
    double can_send_timeout_sec_;
    std::string emergency_stop_topic_{"/emergency_stop"};
    bool require_emergency_stop_{false};
    double emergency_stop_timeout_sec_{0.5};
    double emergency_stop_check_period_sec_{0.05};
    bool require_unique_control_message_publisher_{false};
    SocketCanIntf can_intf_ = SocketCanIntf();

    rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;
    rclcpp::TimerBase::SharedPtr safety_watchdog_timer_;
    EpollEvent safety_evt_;
    std::mutex safety_mutex_;
    bool emergency_stop_received_{false};
    bool emergency_stop_active_{true};
    std::chrono::steady_clock::time_point emergency_stop_updated_at_{};
    std::size_t emergency_stop_publisher_count_{0};
    std::size_t control_message_publisher_count_{0};
    bool commands_authorized_{true};
    uint64_t authorization_generation_{0};
    std::atomic_bool commands_authorized_snapshot_{true};
    std::atomic_uint64_t authorization_generation_snapshot_{0};
    bool safety_resources_initialized_{false};

    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;

    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    uint64_t ctrl_msg_authorization_generation_{0};
    rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;

    EpollEvent srv_evt_;
    std::mutex axis_service_mutex_;
    std::mutex axis_request_queue_mutex_;
    std::deque<std::shared_ptr<PendingAxisSend>> axis_request_queue_;
    uint64_t heartbeat_generation_{0};
    std::chrono::steady_clock::time_point last_heartbeat_time_{};
    std::condition_variable fresh_heartbeat_;
    rclcpp::Service<AxisState>::SharedPtr service_;

    EpollEvent srv_clear_errors_evt_;
    std::mutex clear_errors_queue_mutex_;
    std::deque<std::shared_ptr<PendingCanSend>> clear_errors_queue_;
    rclcpp::Service<Empty>::SharedPtr service_clear_errors_;

    EpollEvent srv_set_configs_evt_;
    std::mutex set_configs_queue_mutex_;
    std::deque<std::shared_ptr<PendingConfigSend>> set_configs_queue_;
    rclcpp::Service<SetConfigs>::SharedPtr service_set_configs_;
};

#endif // ODRIVE_CAN_NODE_HPP
