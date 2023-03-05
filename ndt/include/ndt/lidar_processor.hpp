#ifndef NDT_LIDAR_PROCESSING_
#define NDT_LIDAR_PROCESSING_

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarProcessor : public rclcpp::Node {
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

 public:
  LidarProcessor();

 private:
  void rawscan_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg
  );
  void initialize_parameters();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rawscan_sub_;

  std::string scan_topic_, rawscan_topic_;
  float min_range_, max_range_;
  float leafsize_;
};

#endif
