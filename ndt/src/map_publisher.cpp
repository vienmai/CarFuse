#include "ndt/map_publisher.hpp"

#include <pcl_conversions/pcl_conversions.h>

MapPublisher::MapPublisher() : Node("map_publisher") {
  auto map_topic = declare_parameter("map_topic", std::string("/map"));
  get_parameter("map_topic", map_topic);
  RCLCPP_INFO(get_logger(), "Map topic: %s", map_topic.c_str());
  map_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 1);
}

void MapPublisher::publish_map(const std::string &path) {
  auto map_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (pcl::io::loadPCDFile(path, *map_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to read map file %s", path.c_str());
    return;
  }

  // Convert the PointCloud to a PointCloud2 message
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  auto map_frame = declare_parameter("map_frame", std::string("map_frame"));
  get_parameter("map_frame", map_frame);
  map_msg.header.frame_id = map_frame;
  RCLCPP_INFO(get_logger(), "Map message header id: %s", map_frame.c_str());

  // Publish the map message
  map_pub_->publish(map_msg);
  RCLCPP_INFO(get_logger(), "Finished publishing the map!");
}
