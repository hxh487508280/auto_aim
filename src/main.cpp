#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "auto_aim/auto_aim_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_aim::AutoAimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
