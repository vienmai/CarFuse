#ifndef NDT_MAP_PUBLISHER_
#define NDT_MAP_PUBLISHER_

#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class MapPublisher : public rclcpp::Node {
 public:
  MapPublisher();

 private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
  void load_map(const std::string &path);
  void initialize_parameters();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_;

  std::string map_file_;
  std::string map_frame_;
  std::string map_topic_, pose_topic_;

  Eigen::Vector3d prev_position_;

  bool first_submap_{true};
  double travel_dist_{0};
  double map_update_threshold_;

  double submap_size_xy_, submap_size_z_;
};

#endif
