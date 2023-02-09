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
  using PointT = pcl::PointXYZ;
  using PointPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

 public:
  MapPublisher();
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);

 private:
  void initialize_parameters();
  void load_and_publish_map(const std::string& path);
  PointPtr create_submap(Eigen::Vector3d& curr_position) const;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  PointPtr map_ptr_;

  std::string map_file_;
  std::string map_frame_;
  std::string map_topic_, pose_topic_;

  Eigen::Vector3d prev_position_;

  bool first_submap_{true};
  float travel_dist_{0};
  float map_update_threshold_;
  float submap_size_xy_, submap_size_z_;
};

#endif
