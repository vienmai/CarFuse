#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include "ndt/map_publisher.hpp"

using namespace localization;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance the MapPublisher node
  auto map_publisher_node = std::make_shared<MapPublisher>();

  // Read the configuration file
  std::string map_file = map_publisher_node->get_parameter_or(
      "map_file", std::string("/ros2_ws/install/ndt_locator/share/ndt_locator/"
                              "maps/court_yard_map.pcd")
  );
  RCLCPP_INFO(
      map_publisher_node->get_logger(), "Map file: %s", map_file.c_str()
  );

  // Publish the map to the map topic
  map_publisher_node->publish_map(map_file);

  // Publish the initial pose
  geometry_msgs::msg::PoseStamped initial_pose_msg;
  initial_pose_msg.header.stamp = rclcpp::Clock().now();
  initial_pose_msg.header.frame_id = "map";

  // Set the initial pose here, for example:
  initial_pose_msg.pose.position.x = 0;
  initial_pose_msg.pose.position.y = 0;
  initial_pose_msg.pose.position.z = 0;
  initial_pose_msg.pose.orientation.x = 0;
  initial_pose_msg.pose.orientation.y = 0;
  initial_pose_msg.pose.orientation.z = 0;
  initial_pose_msg.pose.orientation.w = 1;

  // Publish the initial pose
  auto initial_pose_publisher =
      ndt_node->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/initial_pose", 1
      );
  initial_pose_publisher->publish(initial_pose_msg);
  RCLCPP_INFO(ndt_node->get_logger(), "Initial pose published.");

  rclcpp::spin(ndt_node);
  rclcpp::shutdown();

  return 0;
}
