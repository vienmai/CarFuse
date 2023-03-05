#include "ndt/lidar_processor.hpp"
#include "ndt/utility.hpp"

LidarProcessor::LidarProcessor() : Node("lidar_processor") {
  initialize_parameters();

  rawscan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      rawscan_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&LidarProcessor::rawscan_callback, this, std::placeholders::_1)
  );

  scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));
}

void LidarProcessor::rawscan_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr rawscan_msg
) {
  RCLCPP_INFO(get_logger(), "Start raw scan callback.");

  auto rawscan = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*rawscan_msg, *rawscan);
  RCLCPP_INFO(get_logger(), "Received %ld points", rawscan->points.size());

  // Range filter
  auto rfscan = std::make_shared<PointCloud>();
  utility::range_filter(rawscan, rfscan, min_range_, max_range_);
  RCLCPP_INFO(
      get_logger(), "Range filtered: %ld points", rfscan->points.size()
  );

  // Voxel filter
  auto vfscan = std::make_shared<PointCloud>();
  utility::voxel_filter(rfscan, vfscan, leafsize_);
  RCLCPP_INFO(
      get_logger(), "Filtered cloud: %ld points", vfscan->points.size()
  );

  // Publish filtered scan
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*vfscan, scan_msg);
  scan_msg.header = rawscan_msg->header;
  scan_pub_->publish(scan_msg);
}

void LidarProcessor::initialize_parameters() {
  // Filters
  declare_parameter("leafsize", leafsize_);
  declare_parameter("min_range", min_range_);
  declare_parameter("max_range", max_range_);

  get_parameter("leafsize", leafsize_);
  get_parameter("min_range", min_range_);
  get_parameter("max_range", max_range_);

  // Topics
  declare_parameter("scan_topic", scan_topic_);
  declare_parameter("rawscan_topic", rawscan_topic_);

  get_parameter("scan_topic", scan_topic_);
  get_parameter("rawscan_topic", rawscan_topic_);

  RCLCPP_INFO(get_logger(), "rawscan topic: %s", rawscan_topic_.c_str());
  RCLCPP_INFO(get_logger(), "scan topic: %s", scan_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Leaf size: %f", leafsize_);
  RCLCPP_INFO(get_logger(), "Min range: %f", min_range_);
  RCLCPP_INFO(get_logger(), "Max range: %f", max_range_);
}