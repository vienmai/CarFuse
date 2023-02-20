#include "ndt/ndt.hpp"
#include "ndt/utility.hpp"

namespace localization {

NDTLocalization::NDTLocalization() : Node("ndt"), tf_buffer_(), ndt_(new NDT) {
  initialize_parameters();
  initialize_pubsubs();
};

void NDTLocalization::scan_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg
) {
  RCLCPP_INFO(get_logger(), "Start scan callback.");

  if (!initial_pose_received_ || !map_received_) {
    RCLCPP_INFO(get_logger(), "No map or initial pose received.");
    return;
  };

  // Convert the cloud message to a point cloud
  auto cloud = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*pcd_msg, *cloud);
  RCLCPP_INFO(get_logger(), "Received %ld points", cloud->points.size());

  // Filtering input scan to reduce size
  auto scans = std::make_shared<PointCloud>();
  utility::range_filter(cloud, scans, 5, 60);

  // Transform the cloud from sensor to vehicle frame
  auto tfcloud = std::make_shared<PointCloud>();
  transform_pointcloud(
      *scans, *tfcloud, vehicle_frame_, pcd_msg->header.frame_id
  );

  auto voxel_cloud = std::make_shared<PointCloud>();
  utility::voxel_filter(tfcloud, voxel_cloud, leafsize_);
  RCLCPP_INFO(
      get_logger(), "Filtered cloud: %ld points", voxel_cloud->points.size()
  );

  // Perform scan matching
  const std::lock_guard<std::mutex> lock(ndt_mtx_);
  if (ndt_->getInputTarget() == nullptr) {
    RCLCPP_ERROR(get_logger(), "No input target!");
    return;
  }
  ndt_->setInputSource(voxel_cloud);
  auto outcloud = std::make_shared<PointCloud>();
  ndt_->align(*outcloud, current_pose_);

  std::cout << "has converged:" << ndt_->hasConverged() << " score: " <<
  ndt_->getFitnessScore() << " niters: " << ndt_->getFinalNumIteration() << std::endl;

  // Update current pose
  current_pose_ = ndt_->getFinalTransformation();

  RCLCPP_INFO(get_logger(), "Num scan processed: %d", ++index_);

  // Publish the results
  auto stamp = pcd_msg->header.stamp;
  auto pose_msg = utility::eigen_to_pose_stamped(current_pose_, map_frame_, stamp);
  pose_pub_->publish(pose_msg);
  publish_transform(pose_msg);
  publish_path(pose_msg);
  publish_cloud(cloud, stamp, current_pose_);

  RCLCPP_INFO(get_logger(), "End scan callback.");
}

void NDTLocalization::map_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr map_msg
) {
  RCLCPP_INFO(get_logger(), "Start map callback.");

  const auto trans_epsilon = ndt_->getTransformationEpsilon();
  const auto step_size = ndt_->getStepSize();
  const auto resolution = ndt_->getResolution();
  const auto max_iterations = ndt_->getMaximumIterations();

  auto ndt_new = std::make_shared<NDT>();

  ndt_new->setTransformationEpsilon(trans_epsilon);
  ndt_new->setStepSize(step_size);
  ndt_new->setResolution(resolution);
  ndt_new->setMaximumIterations(max_iterations);

  // Set the new map as the target cloud
  auto map_ptr = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*map_msg, *map_ptr);
  ndt_new->setInputTarget(map_ptr);
  RCLCPP_INFO(get_logger(), "Received submap: %ld points", map_ptr->points.size());

  // Dumm alignment to ...
  auto outcloud = std::make_shared<PointCloud>();
  ndt_new->align(*outcloud);

  // Reset ndt_
  ndt_mtx_.lock();
  ndt_ = ndt_new;
  ndt_mtx_.unlock();

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

  current_pose_ = utility::pose_stamped_to_eigen(pose_msg);
  pose_pub_->publish(pose_msg);
  publish_path(pose_msg);

  initial_pose_received_ = true;

  RCLCPP_INFO(get_logger(), "End initial pose callback.");
}

void NDTLocalization::transform_pointcloud(
    const PointCloud &incloud, PointCloud &outcloud,
    const std::string &target_frame, const std::string &source_frame
) const {
  RCLCPP_INFO(get_logger(), "Target: %s, Source: %s", target_frame.c_str(), source_frame.c_str());
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }
  auto eigen_matrix = utility::tf_stamped_to_eigen(tf_stamped);
  pcl::transformPointCloud(incloud, outcloud, eigen_matrix);
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
    const PointCloud::Ptr &cloud, const rclcpp::Time &stamp,
    const Eigen::Matrix4f &pose
) {
  auto tfcloud = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*cloud, *tfcloud, pose);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*tfcloud, cloud_msg);
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

  ndt_->setResolution(resolution_);
  ndt_->setStepSize(stepsize_);
  ndt_->setTransformationEpsilon(epsilon_);
  ndt_->setMaximumIterations(maxiters_);

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

  RCLCPP_INFO(get_logger(), "NDT num iters: %d", ndt_->getMaximumIterations());
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
      scan_topic_, rclcpp::QoS(rclcpp::KeepLast(100)),
      std::bind(&NDTLocalization::scan_callback, this, std::placeholders::_1)
  );

  // Initialize the subscriber for the map
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&NDTLocalization::map_callback, this, std::placeholders::_1)
  );
}

};  // namespace localization
