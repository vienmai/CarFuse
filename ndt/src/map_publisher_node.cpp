#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ndt/map_publisher.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance the MapPublisher node
  auto node = std::make_shared<MapPublisher>();

  // Generate an initial pose
  geometry_msgs::msg::PoseStamped initial_pose_msg;
  initial_pose_msg.header.stamp = rclcpp::Clock().now();
  initial_pose_msg.header.frame_id = "map";
  initial_pose_msg.pose.position.x = 0.0194567;
  initial_pose_msg.pose.position.y = 7.52989e-05;
  initial_pose_msg.pose.position.z = 8.41446e-06;
  initial_pose_msg.pose.orientation.x = 0.00275515;
  initial_pose_msg.pose.orientation.y = -0.0027352;
  initial_pose_msg.pose.orientation.z = 7.53595e-06;
  initial_pose_msg.pose.orientation.w = 0.999992;

  // Publish the initial pose
  auto initial_pose_publisher =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/initial_pose", 1
      );
  initial_pose_publisher->publish(initial_pose_msg);
  RCLCPP_INFO(node->get_logger(), "Initial pose published.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
