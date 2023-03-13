#ifndef NDT_UTILITY_HPP_
#define NDT_UTILITY_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace utility {
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

geometry_msgs::msg::TransformStamped identity_tf_stamped(
    const std::string &header_frame_id, const std::string &child_frame_id,
    const rclcpp::Time &stamp
);

geometry_msgs::msg::PoseStamped eigen_to_pose_stamped(
    const Eigen::Matrix4f &pose, const std::string &frame_id,
    const rclcpp::Time &stamp
);

Eigen::Matrix4f pose_stamped_to_eigen(
    const geometry_msgs::msg::PoseStamped &pose_msg
);

Eigen::Matrix4f tf_stamped_to_eigen(
    const geometry_msgs::msg::TransformStamped &tf_stamped
);

void range_filter(
    const PointCloudPtr incloud, PointCloudPtr outcloud, const float min_range,
    const float max_range
);

void voxel_filter(
    const PointCloudPtr incloud, PointCloudPtr outcloud, const float leafsize
);

void printpose(const Eigen::Matrix4f &pose);

}  // namespace utility
#endif  // NDT_UTILITY_HPP_