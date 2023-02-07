#include <rclcpp/rclcpp.hpp>

#include "ndt/ndt.hpp"

using namespace localization;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto ndt_node = std::make_shared<NDTLocalization>();
  rclcpp::spin(ndt_node);
  rclcpp::shutdown();
  return 0;
}
