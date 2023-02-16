#include "ndt/ndt.hpp"

namespace localization {
NDTLocalization::NDTLocalization() : Node("ndt"), tf_buffer_() {
  initialize_parameters();
  initialize_pubsubs();
};

void NDTLocalization::scan_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg
) {
  RCLCPP_INFO(get_logger(), "Start scan callback.");

  if (!initial_pose_received_ || !map_received_) {
    return;
  };

  // Convert the cloud message to a point cloud
  auto input_cloud = std::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pcd_msg, *input_cloud);
  RCLCPP_INFO(get_logger(), "Converted pcd msg to pcd.");

  // Transform the cloud from sensor to vehicle frame
  transform_pointcloud(
      *input_cloud, *input_cloud, vehicle_frame_, pcd_msg->header.frame_id
  );
  RCLCPP_INFO(get_logger(), "PCD transformed to vehicle frame.");

  // Filtering input scan to reduce size
  auto filtered_cloud = std::make_shared<PointCloudT>();
  voxel_filter(input_cloud, filtered_cloud, leafsize_);
  RCLCPP_INFO(
      get_logger(), "Filtered cloud: %ld points", filtered_cloud->points.size()
  );

  // Perform scan matching
  const std::lock_guard<std::mutex> lock(ndt_mtx_);
  ndt_.setInputSource(filtered_cloud);
  if (ndt_.getInputTarget() == nullptr) {
    RCLCPP_ERROR(get_logger(), "No input target!");
    return;
  }
  auto output_cloud = std::make_shared<PointCloudT>();
  ndt_.align(*output_cloud, current_pose_);
  RCLCPP_INFO(get_logger(), "Finished ndt. Status: %d", ndt_.hasConverged());

  // Update the current pose
  current_pose_ = ndt_.getFinalTransformation();
  Eigen::Matrix3f rmat = current_pose_.block<3, 3>(0, 0);
  Eigen::Vector3f tvec = current_pose_.block<3, 1>(0, 3);
  Eigen::Quaternionf quat(rmat);

  // Create a pose message to publish
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = pcd_msg->header.stamp;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose.position.x = tvec(0);
  pose_msg.pose.position.y = tvec(1);
  pose_msg.pose.position.z = tvec(2);
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();

  // Publish the results
  pose_pub_->publish(pose_msg);
  publish_transform(pose_msg);
  publish_path(pose_msg);
  publish_cloud(output_cloud, pcd_msg->header.stamp);

  RCLCPP_INFO(get_logger(), "End scan callback.");
}

void NDTLocalization::map_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr map_msg
) {
  RCLCPP_INFO(get_logger(), "Start map callback.");

  // const auto trans_epsilon = ndt_.getTransformationEpsilon();
  // const auto step_size = ndt_.getStepSize();
  // const auto resolution = ndt_.getResolution();
  // const auto max_iterations = ndt_.getMaximumIterations();

  // pcl::NormalDistributionsTransform<PointT, PointT> ndt_new;

  // ndt_new.setTransformationEpsilon(trans_epsilon);
  // ndt_new.setStepSize(step_size);
  // ndt_new.setResolution(resolution);
  // ndt_new.setMaximumIterations(max_iterations);

  // Set the new map as the target cloud
  auto map_ptr = std::make_shared<PointCloudT>();
  pcl::fromROSMsg(*map_msg, *map_ptr);
  ndt_.setInputTarget(map_ptr);
  // ndt_new.setInputTarget(map_ptr);

  // Dummy alignment to ...
  // auto output_cloud = std::make_shared<PointCloudT>();
  // ndt_new.align(*output_cloud);

  // Reset ndt_
  // ndt_mutex_.lock();
  // ndt_ = std::move(ndt_new);
  // ndt_mutex_.unlock();

  map_received_ = true;

  RCLCPP_INFO(get_logger(), "End map callback.");
}

void NDTLocalization::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        initial_pose_msg
) {
  RCLCPP_INFO(get_logger(), "Start intitial pose callback.");

  auto initpose_frame = initial_pose_msg->header.frame_id;
  RCLCPP_INFO(get_logger(), "Initial pose frame: %s", initpose_frame.c_str());

  if (initpose_frame != map_frame_) {
    RCLCPP_INFO(
        get_logger(), "Transform initial pose to %s frame", map_frame_.c_str()
    );
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_.lookupTransform(
          map_frame_, initpose_frame, tf2::TimePointZero
      );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
    tf2::doTransform(*initial_pose_msg, *initial_pose_msg, tf_stamped);
  }

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.header.stamp = initial_pose_msg->header.stamp;
  pose_msg.pose = initial_pose_msg->pose.pose;

  current_pose_ = pose_stamped_to_eigen(pose_msg);
  pose_pub_->publish(pose_msg);
  publish_path(pose_msg);

  initial_pose_received_ = true;

  RCLCPP_INFO(get_logger(), "End initial pose callback.");
}

void NDTLocalization::voxel_filter(
    const PointCloudPtr cloud_in, PointCloudPtr cloud_out, const float leafsize
) const {
  pcl::ApproximateVoxelGrid<PointT> voxel_filter;
  voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
  voxel_filter.setInputCloud(cloud_in);
  voxel_filter.filter(*cloud_out);
}

void NDTLocalization::transform_pointcloud(
    const PointCloudT &cloud_in, PointCloudT &cloud_out,
    const std::string &target_frame, const std::string &source_frame
) const {
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }
  auto eigen_matrix = tf_stamped_to_eigen(tf_stamped);
  pcl::transformPointCloud(cloud_in, cloud_out, eigen_matrix);
}

void NDTLocalization::publish_transform(
    geometry_msgs::msg::PoseStamped &pose_msg
) const {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = pose_msg.header.stamp;
  tf_msg.header.frame_id = pose_msg.header.frame_id;
  tf_msg.child_frame_id = vehicle_frame_;

  tf_msg.transform.translation.x = pose_msg.pose.position.x;
  tf_msg.transform.translation.y = pose_msg.pose.position.y;
  tf_msg.transform.translation.z = pose_msg.pose.position.z;
  tf_msg.transform.rotation.x = pose_msg.pose.orientation.x;
  tf_msg.transform.rotation.y = pose_msg.pose.orientation.y;
  tf_msg.transform.rotation.z = pose_msg.pose.orientation.z;
  tf_msg.transform.rotation.w = pose_msg.pose.orientation.w;

  tf_broadcaster_->sendTransform(tf_msg);
}

void NDTLocalization::publish_cloud(
    const PointCloudT::Ptr &cloud, const rclcpp::Time &stamp
) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = map_frame_;
  cloud_msg.header.stamp = stamp;
  cloud_pub_->publish(cloud_msg);
}

void NDTLocalization::publish_path(geometry_msgs::msg::PoseStamped &pose_msg) {
  path_.header.frame_id = pose_msg.header.frame_id;
  path_.poses.push_back(pose_msg);
  path_pub_->publish(path_);
  RCLCPP_INFO(get_logger(), "Path published to %s topic!", path_topic_.c_str());
}

Eigen::Matrix4f NDTLocalization::pose_stamped_to_eigen(
    const geometry_msgs::msg::PoseStamped &pose_msg
) const {
  Eigen::Affine3d affine;
  tf2::fromMsg(pose_msg.pose, affine);
  Eigen::Matrix4f eigen_matrix = affine.matrix().cast<float>();
  return eigen_matrix;
}

Eigen::Matrix4f NDTLocalization::tf_stamped_to_eigen(
    const geometry_msgs::msg::TransformStamped &tf_stamped
) const {
  auto affine = tf2::transformToEigen(tf_stamped);
  Eigen::Matrix4f eigen_matrix = affine.matrix().cast<float>();
  return eigen_matrix;
}

void NDTLocalization::initialize_parameters() {
  // NDT Matcher
  declare_parameter("resolution", resolution_);
  declare_parameter("stepsize", stepsize_);
  declare_parameter("epsilon", epsilon_);
  declare_parameter("maxiters", maxiters_);
  declare_parameter("leafsize", leafsize_);

  get_parameter("resolution", resolution_);
  get_parameter("stepsize", stepsize_);
  get_parameter("epsilon", epsilon_);
  get_parameter("maxiters", maxiters_);
  get_parameter("leafsize", leafsize_);

  ndt_.setResolution(resolution_);
  ndt_.setStepSize(stepsize_);
  ndt_.setTransformationEpsilon(epsilon_);
  ndt_.setMaximumIterations(maxiters_);

  // Reference Frames
  declare_parameter("map_frame", map_frame_);
  declare_parameter("vehicle_frame", vehicle_frame_);
  declare_parameter("laser_frame", laser_frame_);
  declare_parameter("initial_pose_frame", initial_pose_frame_);

  get_parameter("map_frame", map_frame_);
  get_parameter("vehicle_frame", vehicle_frame_);
  get_parameter("laser_frame", laser_frame_);
  get_parameter("initial_pose_frame", initial_pose_frame_);

  // Topics
  declare_parameter("scan_topic", scan_topic_);
  declare_parameter("initial_pose_topic", initial_pose_topic_);
  declare_parameter("map_topic", map_topic_);
  declare_parameter("pose_topic", pose_topic_);
  declare_parameter("path_topic", path_topic_);
  declare_parameter("cloud_topic", cloud_topic_);
  declare_parameter("tf_topic", tf_topic_);

  get_parameter("scan_topic", scan_topic_);
  get_parameter("initial_pose_topic", initial_pose_topic_);
  get_parameter("map_topic", map_topic_);
  get_parameter("pose_topic", pose_topic_);
  get_parameter("path_topic", path_topic_);
  get_parameter("cloud_topic", cloud_topic_);
  get_parameter("tf_topic", tf_topic_);
}

void NDTLocalization::initialize_pubsubs() {
  // TF2 buffer and listener
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // Publisher for the current pose
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(rclcpp::KeepLast(10))
  );

  // Publisher for the aligned point cloud in map frame
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(10))
  );

  // Publisher for the estimated path
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(rclcpp::KeepLast(10))
  );

  // Subscriber for the initial pose
  initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initial_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
          std::bind(
              &NDTLocalization::initial_pose_callback, this,
              std::placeholders::_1
          )
      );

  // Initialize the subscriber for the lidar scan
  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&NDTLocalization::scan_callback, this, std::placeholders::_1)
  );

  // Initialize the subscriber for the map
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&NDTLocalization::map_callback, this, std::placeholders::_1)
  );
}

};  // namespace localization
