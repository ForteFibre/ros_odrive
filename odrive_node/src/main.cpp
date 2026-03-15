#include "odrive_can_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");

  if (!can_node->init()) return -1;

  rclcpp::spin(can_node);
  can_node->deinit();
  rclcpp::shutdown();
  return 0;
}
