#ifndef NDT_NDT_HPP_
#define NDT_NDT_HPP_

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace localization {
class NDTLocalization : public rclcpp::Node {
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

 public:
  NDTLocalization();

 private:
  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg);
  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg);
  void initial_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
          initial_pose_msg
  );
  void initialize_parameters();
  void initialize_pubsubs();

  /** @brief Publish the transformation between the laser and the map.
   * @param pose: The estimated pose in the map frame
   */
  void publish_transform(geometry_msgs::msg::PoseStamped &pose_msg) const;

  /** @brief Publish the estimated path.
   * @param pose_msg: The estimated pose in the map frame
   */
  void publish_path(geometry_msgs::msg::PoseStamped &pose_msg);

  /** @brief Publish the point cloud.
   * @param cloud: The cloud to be published.
   * @param stamp: The time at which this pose was calculated. Default: 0.
   */
  void publish_cloud(const PointCloudT::Ptr &cloud, const rclcpp::Time &stamp);

  /** @brief Transform the input point cloud to the target frame
   * @param cloud_in: The input point cloud.
   * @param cloud_out: The output transfomred point cloud.
   * @param target_frame: The name of the target frame.
   */
  void transform_pointcloud(
      const PointCloudT &cloud_in, PointCloudT &cloud_out,
      const std::string &target_frame, const std::string &source_frame
  ) const;

  void voxel_filter(
      const PointCloudPtr cloud_in, PointCloudPtr cloud_out,
      const float leafsize
  ) const;

  Eigen::Matrix4f pose_stamped_to_eigen(
      const geometry_msgs::msg::PoseStamped &pose_msg
  ) const;

  /** @brief Transform tf2 stamped to Eigen::Matrix4f.
   * @param tf_stamped: The input transform stamped
   */
  Eigen::Matrix4f tf_stamped_to_eigen(
      const geometry_msgs::msg::TransformStamped &tf_stamped
  ) const;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;

  Eigen::Matrix4f current_pose_;
  nav_msgs::msg::Path path_;

  std::string map_frame_, vehicle_frame_, laser_frame_, initial_pose_frame_;
  std::string scan_topic_, initial_pose_topic_, map_topic_, pose_topic_,
      path_topic_, cloud_topic_, tf_topic_;

  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  double resolution_;
  double stepsize_;
  double epsilon_;
  int maxiters_;
  double leafsize_;
};
};  // namespace localization

#endif  // NDT_NDT_HPP_
