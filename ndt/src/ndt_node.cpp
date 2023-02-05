#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ndt/ndt.hpp"

using namespace localization;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance of the NDTLocalization class
  auto ndt_node = std::make_shared<NDTLocalization>();

  // Create a publisher for the estimated poses
  auto pose_publisher =
      ndt_node->create_publisher<geometry_msgs::msg::PoseArray>("/viz/pose", 1);

  // Spin to receive sensor data and update estimated poses
  rclcpp::spin(ndt_node);
  rclcpp::shutdown();

  // Collect all estimated poses and publish them once
  auto estimated_poses = ndt_node->get_estimated_poses();
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";
  for (const auto &pose : estimated_poses) {
    msg.poses.push_back(pose.pose);
  }
  pose_publisher->publish(msg);

  return 0;
}
