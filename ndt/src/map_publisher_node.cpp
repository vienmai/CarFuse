#include <rclcpp/rclcpp.hpp>

#include "ndt/map_publisher.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
