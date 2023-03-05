#include <rclcpp/rclcpp.hpp>

#include "ndt/lidar_processor.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
