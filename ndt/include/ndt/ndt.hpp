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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace localization {
class NDTLocalization : public rclcpp::Node {
  using PointT = pcl::PointXYZ;

 public:
  NDTLocalization();

  /** @brief Vector of estimated poses.*/
  std::vector<geometry_msgs::msg::PoseStamped> get_estimated_poses() const;

 private:
  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg);
  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg);
  void initial_pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr initial_pose_msg
  );
  void initialize_parameters();

  /** @brief Publish the transformation between the laser and the map.
   * @param pose: The pose of the laser in the map frame
   * @param stamp: The time at which this pose was calculated. Default: 0.
   */
  void publish_transform(geometry_msgs::msg::PoseStamped &pose_msg) const;

  /** @brief Publish the point cloud.
   * @param cloud: The cloud to be published.
   * @param stamp: The time at which this pose was calculated. Default: 0.
   */
  void publish_cloud(
      const pcl::PointCloud<PointT>::Ptr &cloud, const rclcpp::Time &stamp
  ) const;

  /** @brief Transform the input point cloud to the target frame
   * @param cloud_in: The input point cloud.
   * @param cloud_out: The output transfomred point cloud.
   * @param target_frame: The name of the target frame.
   */
  void transform_pointcloud(
      const pcl::PointCloud<PointT> &cloud_in,
      pcl::PointCloud<PointT> &cloud_out, const std::string &target_frame
  ) const;

  /** @brief Transform tf2 stamped to Eigen::Matrix4f.
   * @param tf_stamped: The input transform stamped
   */
  Eigen::Matrix4f tf_stamped_to_eigen(
      const geometry_msgs::msg::TransformStamped &tf_stamped
  ) const;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      initial_pose_sub_;

  pcl::PointCloud<PointT>::Ptr map_ptr_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;

  Eigen::Matrix4f current_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> pose_vec_;

  std::string map_frame_, vehicle_frame_, laser_frame_, initial_pose_frame_;
  std::string scan_topic_, initial_pose_topic_, map_topic_, pose_topic_,
      cloud_topic_, tf_topic_;

  double resolution_;
  double stepsize_;
  double epsilon_;
  size_t maxiters_;
  double leafsize_;
};
};  // namespace localization

#endif  // NDT_NDT_HPP_
