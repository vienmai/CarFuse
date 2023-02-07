#include "ndt/map_publisher.hpp"

MapPublisher::MapPublisher() : Node("map_publisher") {
  initialize_parameters();

  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 10);
  this->load_map(map_file_);

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
  Eigen::Vector3d curr_position(pose(0, 3), pose(1, 3), pose(2, 3));

  if (!first_submap_) {
    travel_dist_ += (curr_position - prev_position_).norm();
    RCLCPP_INFO(get_logger(), "Distance traveled: %f", travel_dist_);
  }

  if (travel_dist_ > map_update_threshold_ || first_submap_) {
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(
        curr_position.x() - submap_size_xy_,
        curr_position.y() - submap_size_xy_, curr_position.z() - submap_size_z_,
        1.0
    ));
    box_filter.setMax(Eigen::Vector4f(
        curr_position.x() + submap_size_xy_,
        curr_position.y() + submap_size_xy_, curr_position.z() + submap_size_z_,
        1.0
    ));

    auto submap_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    box_filter.setInputCloud(map_ptr_);
    box_filter.filter(*submap_ptr);

    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*submap_ptr, submap_msg);
    submap_msg.header.frame_id = map_frame_;
    RCLCPP_INFO(
        get_logger(), "Map frame: %s", submap_msg.header.frame_id.c_str()
    );

    if (submap_msg.width > 0) {
      map_pub_->publish(submap_msg);
      RCLCPP_INFO(get_logger(), "New submap publsihed!");
      travel_dist_ = 0;
    }
    first_submap_ = false;
  }

  prev_position_ = curr_position;
}

void MapPublisher::load_map(const std::string &path) {
  auto map_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Load map file
  if (pcl::io::loadPCDFile(path, *map_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to read map file %s", path.c_str());
    return;
  }

  // Convert the PointCloud to a PointCloud2 message
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.frame_id = map_frame_;
  RCLCPP_INFO(get_logger(), "Map frame: %s", map_msg.header.frame_id.c_str());

  // Set the pointer to the global map
  map_ptr_ = map_cloud;

  // Publish the map message
  map_pub_->publish(map_msg);
  RCLCPP_INFO(get_logger(), "Published the full map!");
}

void MapPublisher::initialize_parameters() {
  declare_parameter("map_topic", map_topic_);
  declare_parameter("pose_topic", pose_topic_);

  get_parameter("map_topic", map_topic_);
  get_parameter("pose_topic", pose_topic_);

  declare_parameter("map_frame", map_frame_);
  get_parameter("map_frame", map_frame_);
  RCLCPP_INFO(get_logger(), "Map frame: %s", map_frame_.c_str());

  declare_parameter("map_file", map_file_);
  get_parameter("map_file", map_file_);
  RCLCPP_INFO(get_logger(), "Map file: %s", map_file_.c_str());

  declare_parameter("map_update_threshold", map_update_threshold_);
  declare_parameter("submap_size_xy", submap_size_xy_);
  declare_parameter("submap_size_z", submap_size_z_);

  get_parameter("map_update_threshold", map_update_threshold_);
  get_parameter("submap_size_xy", submap_size_xy_);
  get_parameter("submap_size_z", submap_size_z_);

  RCLCPP_INFO(get_logger(), "Map update threshold: %f", map_update_threshold_);
  RCLCPP_INFO(get_logger(), "submap_size_xy: %f", submap_size_xy_);
  RCLCPP_INFO(get_logger(), "submap_size_z: %f", submap_size_z_);
}
