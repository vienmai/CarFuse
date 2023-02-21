#include "ndt/map_publisher.hpp"

MapPublisher::MapPublisher() : Node("map_publisher") {
  initialize_parameters();

  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 10);
  this->loadmap(map_file_);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&MapPublisher::pose_callback, this, std::placeholders::_1)
  );
}

void MapPublisher::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg
) {
  Eigen::Affine3d pose;
  tf2::fromMsg(pose_msg->pose, pose);
  Eigen::Vector3f curr_position(pose(0, 3), pose(1, 3), pose(2, 3));

  if (!first_submap_) {
    travel_dist_ += (curr_position - prev_position_).norm();
  }
  RCLCPP_INFO(get_logger(), "Distance traveled: %f", travel_dist_);

  if (first_submap_ || travel_dist_ > map_update_threshold_) {
    auto submap_ptr = create_submap(curr_position);
    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*submap_ptr, submap_msg);
    submap_msg.header.frame_id = map_frame_;

    if (submap_msg.width > 0) {
      map_pub_->publish(submap_msg);
      RCLCPP_INFO(get_logger(), "New submap published!");
      travel_dist_ = 0;
    }
    first_submap_ = false;
  }
  prev_position_ = curr_position;
}

void MapPublisher::loadmap(const std::string& path) {
  auto map_cloud = std::make_shared<PointCloudT>();

  if (pcl::io::loadPCDFile(path, *map_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to read map file %s", path.c_str());
    return;
  }

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.frame_id = map_frame_;

  // Set the pointer to the global map
  map_ptr_ = map_cloud;
  RCLCPP_INFO(get_logger(), "Full map: %ld points", map_ptr_->points.size());
}

MapPublisher::PointCloudPtr MapPublisher::create_submap(
    Eigen::Vector3f& curr_position
) const {
  pcl::CropBox<PointT> box_filter;
  box_filter.setMin(Eigen::Vector4f(
      curr_position.x() - submap_size_xy_, curr_position.y() - submap_size_xy_,
      curr_position.z() - submap_size_z_, 1.0
  ));
  box_filter.setMax(Eigen::Vector4f(
      curr_position.x() + submap_size_xy_, curr_position.y() + submap_size_xy_,
      curr_position.z() + submap_size_z_, 1.0
  ));

  auto submap_ptr = std::make_shared<PointCloudT>();
  box_filter.setInputCloud(map_ptr_);
  box_filter.filter(*submap_ptr);
  RCLCPP_INFO(
      get_logger(), "Submap created: %ld points", submap_ptr->points.size()
  );

  return submap_ptr;
}

void MapPublisher::initialize_parameters() {
  declare_parameter("map_topic", map_topic_);
  declare_parameter("pose_topic", pose_topic_);

  get_parameter("map_topic", map_topic_);
  get_parameter("pose_topic", pose_topic_);

  declare_parameter("map_frame", map_frame_);
  get_parameter("map_frame", map_frame_);

  declare_parameter("map_file", map_file_);
  get_parameter("map_file", map_file_);

  declare_parameter("map_update_threshold", map_update_threshold_);
  declare_parameter("submap_size_xy", submap_size_xy_);
  declare_parameter("submap_size_z", submap_size_z_);

  get_parameter("map_update_threshold", map_update_threshold_);
  get_parameter("submap_size_xy", submap_size_xy_);
  get_parameter("submap_size_z", submap_size_z_);

  RCLCPP_INFO(get_logger(), "Map file: %s", map_file_.c_str());
  RCLCPP_INFO(get_logger(), "Map frame: %s", map_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Map update threshold: %f", map_update_threshold_);
  RCLCPP_INFO(get_logger(), "submap_size_xy: %f", submap_size_xy_);
  RCLCPP_INFO(get_logger(), "submap_size_z: %f", submap_size_z_);
}
