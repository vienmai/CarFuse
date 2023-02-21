#include "ndt/utility.hpp"

namespace utility {

geometry_msgs::msg::PoseStamped eigen_to_pose_stamped(
    const Eigen::Matrix4f &pose, const std::string &frame_id,
    const rclcpp::Time &stamp
) {
  Eigen::Matrix3f rmat = pose.block<3, 3>(0, 0);
  Eigen::Vector3f tvec = pose.block<3, 1>(0, 3);
  Eigen::Quaternionf quat(rmat);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = frame_id;
  pose_msg.pose.position.x = tvec(0);
  pose_msg.pose.position.y = tvec(1);
  pose_msg.pose.position.z = tvec(2);
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();

  return pose_msg;
}

Eigen::Matrix4f pose_stamped_to_eigen(
    const geometry_msgs::msg::PoseStamped &pose_msg
) {
  Eigen::Affine3d affine;
  tf2::fromMsg(pose_msg.pose, affine);
  Eigen::Matrix4f eigen_matrix = affine.matrix().cast<float>();
  return eigen_matrix;
}

Eigen::Matrix4f tf_stamped_to_eigen(
    const geometry_msgs::msg::TransformStamped &tf_stamped
) {
  auto affine = tf2::transformToEigen(tf_stamped);
  Eigen::Matrix4f eigen_matrix = affine.matrix().cast<float>();
  return eigen_matrix;
}

void range_filter(
    const PointCloudPtr incloud, PointCloudPtr outcloud, const float min_range,
    const float max_range
) {
  for (const auto &point : *incloud) {
    auto dist = std::sqrt(std::pow(point.x, 2.0) + std::pow(point.y, 2.0));
    if (min_range <= dist && dist <= max_range) {
      outcloud->push_back(point);
    }
  }
}

void voxel_filter(
    const PointCloudPtr incloud, PointCloudPtr outcloud, const float leafsize
) {
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
  voxel_filter.setInputCloud(incloud);
  voxel_filter.filter(*outcloud);
}

void printpcd(const pcl::PointCloud<pcl::PointXYZ> &cloud, const int n) {
  int count = 0;
  for (const auto &point : cloud) {
    std::cout << point << std::endl;
    if (++count >= n) break;
  }
}

void printpose(const Eigen::Matrix4f &pose) {
  printf("\n");
  pcl::console::print_info(
      "    | %6.3f %6.3f %6.3f | \n", pose(0, 0), pose(0, 1), pose(0, 2)
  );
  pcl::console::print_info(
      "R = | %6.3f %6.3f %6.3f | \n", pose(1, 0), pose(1, 1), pose(1, 2)
  );
  pcl::console::print_info(
      "    | %6.3f %6.3f %6.3f | \n", pose(2, 0), pose(2, 1), pose(2, 2)
  );
  pcl::console::print_info("\n");
  pcl::console::print_info(
      "t = < %0.3f, %0.3f, %0.3f >\n", pose(0, 3), pose(1, 3), pose(2, 3)
  );
  pcl::console::print_info("\n");
}

}  // namespace utility