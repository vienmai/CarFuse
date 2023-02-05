#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ndt/map_publisher.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance the MapPublisher node
  auto node = std::make_shared<MapPublisher>();

  // Read the configuration file
  auto map_file = node->declare_parameter("map_file", std::string("map.pcd"));
  node->get_parameter("map_file", map_file);
  RCLCPP_INFO(node->get_logger(), "Map file: %s", map_file.c_str());

  // Publish the map to the map topic
  node->publish_map(map_file);

  // Generate an initial pose
  geometry_msgs::msg::PoseStamped initial_pose_msg;
  initial_pose_msg.header.stamp = rclcpp::Clock().now();
  initial_pose_msg.header.frame_id = "map";
  initial_pose_msg.pose.position.x = 0;
  initial_pose_msg.pose.position.y = 0;
  initial_pose_msg.pose.position.z = 0;
  initial_pose_msg.pose.orientation.x = 0;
  initial_pose_msg.pose.orientation.y = 0;
  initial_pose_msg.pose.orientation.z = 0;
  initial_pose_msg.pose.orientation.w = 1;

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
