#include <thread>

#include "epoll_event_loop.hpp"
#include "odrive_can_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "socket_can.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  EpollEventLoop event_loop;
  auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");

  if (!can_node->init(&event_loop)) return -1;

  std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });
  // AxisState intentionally waits for a physical procedure heartbeat. Keep a
  // second executor worker available for the independent safety callback group
  // so E-stop heartbeat loss can still enqueue IDLE during calibration.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2U);
  executor.add_node(can_node);
  executor.spin();
  executor.remove_node(can_node);
  // Deregister every epoll source first so run_until_empty() can terminate,
  // then join before destroying either the event loop or node callbacks.
  can_node->deinit();
  if (can_event_loop.joinable()) {
    can_event_loop.join();
  }
  rclcpp::shutdown();
  return 0;
}
